#pragma once
//#include <opencv2/opencv.hpp>
using namespace std;

#define CV_SINGLETON_LAZY_INIT_(TYPE, INITIALIZER, RET_VALUE) \
    static TYPE* const instance = INITIALIZER; \
    return RET_VALUE;

//#define CV_SINGLETON_LAZY_INIT(TYPE, INITIALIZER) CV_SINGLETON_LAZY_INIT_(TYPE, INITIALIZER, instance)
#define CV_SINGLETON_LAZY_INIT_REF(TYPE, INITIALIZER) CV_SINGLETON_LAZY_INIT_(TYPE, INITIALIZER, *instance)


struct ThreadData
{
    ThreadData()
    {
        idx = 0;
        slots.reserve(32);
    }

    std::vector<void*> slots; // Data array for a thread
    size_t idx;               // Thread index in TLS storage. This is not OS thread ID!
};

class TLSDataContainer;

bool __termination = false;

template <class T>
class DisposedSingletonMark
{
private:
    static bool mark;
protected:
    DisposedSingletonMark() {}
    ~DisposedSingletonMark()
    {
        mark = true;
    }
public:
    static bool isDisposed() { return mark; }
};

// TLS platform abstraction layer
class TlsAbstraction : public DisposedSingletonMark<TlsAbstraction>
{
public:
    TlsAbstraction();
    ~TlsAbstraction();
    void* getData() const
    {
        if (isDisposed())  // guard: static initialization order fiasco
            return NULL;
        return getData_();
    }
    void setData(void* pData)
    {
        if (isDisposed())  // guard: static initialization order fiasco
            return;
        return setData_(pData);
    }

private:
    void* getData_() const;
    void setData_(void* pData);

#ifdef _WIN32
#ifndef WINRT
    DWORD tlsKey;
#endif
#else // _WIN32
    pthread_key_t  tlsKey;
#endif
};

template<> bool DisposedSingletonMark<TlsAbstraction>::mark = false;

static TlsAbstraction& getTlsAbstraction_()
{
    static TlsAbstraction g_tls;  // disposed in atexit() handlers (required for unregistering our callbacks)
    return g_tls;
}
static TlsAbstraction* getTlsAbstraction()
{
    static TlsAbstraction* instance = &getTlsAbstraction_();
    return DisposedSingletonMark<TlsAbstraction>::isDisposed() ? NULL : instance;
}


static __declspec(thread) void* tlsData = NULL;
TlsAbstraction::TlsAbstraction() {}
TlsAbstraction::~TlsAbstraction()
{
    __termination = true;  // DllMain is missing in static builds
}
void* TlsAbstraction::getData_() const
{
    return tlsData;
}
void TlsAbstraction::setData_(void* pData)
{
    tlsData = pData;
}

// Main TLS storage class
class TlsStorage
{
public:
    TlsStorage() :
        tlsSlotsSize(0)
    {
        tlsSlots.reserve(32);
        threads.reserve(32);
    }
    ~TlsStorage()
    {
        // TlsStorage object should not be released
        // There is no reliable way to avoid problems caused by static initialization order fiasco
        // Don't use logging here
        fprintf(stderr, "OpenCV FATAL: TlsStorage::~TlsStorage() call is not expected\n");
        fflush(stderr);
    }

    /*void releaseThread(void* tlsValue = NULL)
    {
        TlsAbstraction* tls = getTlsAbstraction();
        if (NULL == tls)
            return;  // TLS singleton is not available (terminated)
        ThreadData* pTD = tlsValue == NULL ? (ThreadData*)tls->getData() : (ThreadData*)tlsValue;
        if (pTD == NULL)
            return;  // no OpenCV TLS data for this thread
        AutoLock guard(mtxGlobalAccess);
        for (size_t i = 0; i < threads.size(); i++)
        {
            if (pTD == threads[i])
            {
                threads[i] = NULL;
                if (tlsValue == NULL)
                    tls->setData(0);
                std::vector<void*>& thread_slots = pTD->slots;
                for (size_t slotIdx = 0; slotIdx < thread_slots.size(); slotIdx++)
                {
                    void* pData = thread_slots[slotIdx];
                    thread_slots[slotIdx] = NULL;
                    if (!pData)
                        continue;
                    TLSDataContainer* container = tlsSlots[slotIdx].container;
                    if (container)
                        container->deleteDataInstance(pData);
                    else
                    {
                        fprintf(stderr, "OpenCV ERROR: TLS: container for slotIdx=%d is NULL. Can't release thread data\n", (int)slotIdx);
                        fflush(stderr);
                    }
                }
                delete pTD;
                return;
            }
        }
        fprintf(stderr, "OpenCV WARNING: TLS: Can't release thread TLS data (unknown pointer or data race): %p\n", (void*)pTD); fflush(stderr);
    }*/

    // Reserve TLS storage index
    size_t reserveSlot(TLSDataContainer* container)
    {
        AutoLock guard(mtxGlobalAccess);
        CV_Assert(tlsSlotsSize == tlsSlots.size());

        // Find unused slots
        for (size_t slot = 0; slot < tlsSlotsSize; slot++)
        {
            if (tlsSlots[slot].container == NULL)
            {
                tlsSlots[slot].container = container;
                return slot;
            }
        }

        // Create new slot
        tlsSlots.push_back(TlsSlotInfo(container)); tlsSlotsSize++;
        return tlsSlotsSize - 1;
    }

    // Release TLS storage index and pass associated data to caller
    void releaseSlot(size_t slotIdx, std::vector<void*>& dataVec, bool keepSlot = false)
    {
        AutoLock guard(mtxGlobalAccess);
        CV_Assert(tlsSlotsSize == tlsSlots.size());
        CV_Assert(tlsSlotsSize > slotIdx);

        for (size_t i = 0; i < threads.size(); i++)
        {
            if (threads[i])
            {
                std::vector<void*>& thread_slots = threads[i]->slots;
                if (thread_slots.size() > slotIdx && thread_slots[slotIdx])
                {
                    dataVec.push_back(thread_slots[slotIdx]);
                    thread_slots[slotIdx] = NULL;
                }
            }
        }

        if (!keepSlot)
        {
            tlsSlots[slotIdx].container = NULL;  // mark slot as free (see reserveSlot() implementation)
        }
    }

    // Get data by TLS storage index
    void* getData(size_t slotIdx) const
    {
#ifndef CV_THREAD_SANITIZER
        CV_Assert(tlsSlotsSize > slotIdx);
#endif

        TlsAbstraction* tls = getTlsAbstraction();
        if (NULL == tls)
            return NULL;  // TLS singleton is not available (terminated)

        ThreadData* threadData = (ThreadData*)tls->getData();
        if (threadData && threadData->slots.size() > slotIdx)
            return threadData->slots[slotIdx];

        return NULL;
    }

    // Gather data from threads by TLS storage index
    void gather(size_t slotIdx, std::vector<void*>& dataVec)
    {
        AutoLock guard(mtxGlobalAccess);
        CV_Assert(tlsSlotsSize == tlsSlots.size());
        CV_Assert(tlsSlotsSize > slotIdx);

        for (size_t i = 0; i < threads.size(); i++)
        {
            if (threads[i])
            {
                std::vector<void*>& thread_slots = threads[i]->slots;
                if (thread_slots.size() > slotIdx && thread_slots[slotIdx])
                    dataVec.push_back(thread_slots[slotIdx]);
            }
        }
    }

    // Set data to storage index
    void setData(size_t slotIdx, void* pData)
    {
#ifndef CV_THREAD_SANITIZER
        CV_Assert(tlsSlotsSize > slotIdx);
#endif

        TlsAbstraction* tls = getTlsAbstraction();
        if (NULL == tls)
            return;  // TLS singleton is not available (terminated)

        ThreadData* threadData = (ThreadData*)tls->getData();
        if (!threadData)
        {
            threadData = new ThreadData;
            tls->setData((void*)threadData);
            {
                AutoLock guard(mtxGlobalAccess);

                bool found = false;
                // Find unused slots
                for (size_t slot = 0; slot < threads.size(); slot++)
                {
                    if (threads[slot] == NULL)
                    {
                        threadData->idx = (int)slot;
                        threads[slot] = threadData;
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    // Create new slot
                    threadData->idx = threads.size();
                    threads.push_back(threadData);
                }
            }
        }

        if (slotIdx >= threadData->slots.size())
        {
            AutoLock guard(mtxGlobalAccess); // keep synchronization with gather() calls
            threadData->slots.resize(slotIdx + 1, NULL);
        }
        threadData->slots[slotIdx] = pData;
    }

private:
    Mutex  mtxGlobalAccess;           // Shared objects operation guard
    size_t tlsSlotsSize;              // equal to tlsSlots.size() in synchronized sections
    // without synchronization this counter doesn't decrease - it is used for slotIdx sanity checks

    struct TlsSlotInfo
    {
        TlsSlotInfo(TLSDataContainer* _container) : container(_container) {}
        TLSDataContainer* container;  // attached container (to dispose data of terminated threads)
    };
    std::vector<struct TlsSlotInfo> tlsSlots;  // TLS keys state
    std::vector<ThreadData*> threads; // Array for all allocated data. Thread data pointers are placed here to allow data cleanup
};

static TlsStorage& getTlsStorage()
{
    CV_SINGLETON_LAZY_INIT_REF(TlsStorage, new TlsStorage())
}




class CV_EXPORTS TLSDataContainer
{
protected:
    TLSDataContainer();
    virtual ~TLSDataContainer();

    /// @deprecated use detachData() instead
    void  gatherData(std::vector<void*>& data) const;
    /// get TLS data and detach all data from threads (similar to cleanup() call)
    void  detachData(std::vector<void*>& data);

    void* getData() const;
    void  release();

protected:
    virtual void* createDataInstance() const = 0;
    virtual void  deleteDataInstance(void* pData) const = 0;

private:
    int key_;

    friend class TlsStorage;  // core/src/system.cpp

public:
    void cleanup(); //!< Release created TLS data container objects. It is similar to release() call, but it keeps TLS container valid.

private:
    // Disable copy/assign (noncopyable pattern)
    TLSDataContainer(TLSDataContainer&) = delete;
    TLSDataContainer& operator =(const TLSDataContainer&) = delete;
};

TLSDataContainer::TLSDataContainer()
{
    key_ = (int)getTlsStorage().reserveSlot(this); // Reserve key from TLS storage
}

TLSDataContainer::~TLSDataContainer()
{
    CV_Assert(key_ == -1); // Key must be released in child object
}



void TLSDataContainer::gatherData(std::vector<void*>& data) const
{
    getTlsStorage().gather(key_, data);
}


void TLSDataContainer::release()
{
    if (key_ == -1)
        return;  // already released
    std::vector<void*> data; data.reserve(32);
    getTlsStorage().releaseSlot(key_, data, false); // Release key and get stored data for proper destruction
    key_ = -1;
    for (size_t i = 0; i < data.size(); i++)  // Delete all associated data
        deleteDataInstance(data[i]);
}

void TLSDataContainer::cleanup()
{
    std::vector<void*> data; data.reserve(32);
    getTlsStorage().releaseSlot(key_, data, true); // Extract stored data with removal from TLS tables
    for (size_t i = 0; i < data.size(); i++)  // Delete all associated data
        deleteDataInstance(data[i]);
}

void* TLSDataContainer::getData() const
{
    CV_Assert(key_ != -1 && "Can't fetch data from terminated TLS container.");
    void* pData = getTlsStorage().getData(key_); // Check if data was already allocated
    if (!pData)
    {
        // Create new data instance and save it to TLS storage
        pData = createDataInstance();
        getTlsStorage().setData(key_, pData);
    }
    return pData;
}







/** @brief Simple TLS data class
 *
 * @sa TLSDataAccumulator
 */
template <typename T>
class TLSData : protected TLSDataContainer
{
public:
    inline TLSData() {}
    inline ~TLSData() { release(); }

    inline T* get() const { return (T*)getData(); }  //!< Get data associated with key
    inline T& getRef() const { T* ptr = (T*)getData(); CV_DbgAssert(ptr); return *ptr; }  //!< Get data associated with key

    /// Release associated thread data
    inline void cleanup()
    {
        TLSDataContainer::cleanup();
    }

protected:
    /// Wrapper to allocate data by template
    virtual void* createDataInstance() const CV_OVERRIDE { return new T; }
    /// Wrapper to release data by template
    virtual void  deleteDataInstance(void* pData) const CV_OVERRIDE { delete (T*)pData; }
};


template <typename T>
class TLSDataAccumulator : public TLSData<T>
{
    mutable cv::Mutex mutex;
    mutable std::vector<T*> dataFromTerminatedThreads;
    std::vector<T*> detachedData;
    bool cleanupMode;
public:
    TLSDataAccumulator() : cleanupMode(false) {}
    ~TLSDataAccumulator()
    {
        release();
    }

    /** @brief Get data from all threads
     * @deprecated replaced by detachData()
     *
     * Lifetime of vector data is valid until next detachData()/cleanup()/release() calls
     *
     * @param[out] data result buffer (should be empty)
     */
    void gather(std::vector<T*>& data) const
    {
        CV_Assert(cleanupMode == false);  // state is not valid
        CV_Assert(data.empty());
        {
            std::vector<void*>& dataVoid = reinterpret_cast<std::vector<void*>&>(data);
            TLSDataContainer::gatherData(dataVoid);
        }
        {
            AutoLock lock(mutex);
            data.reserve(data.size() + dataFromTerminatedThreads.size());
            for (typename std::vector<T*>::const_iterator i = dataFromTerminatedThreads.begin(); i != dataFromTerminatedThreads.end(); ++i)
            {
                data.push_back((T*)*i);
            }
        }
    }

    /** @brief Get and detach data from all threads
     *
     * Call cleanupDetachedData() when returned vector is not needed anymore.
     *
     * @return Vector with associated data. Content is preserved (including lifetime of attached data pointers) until next detachData()/cleanupDetachedData()/cleanup()/release() calls
     */
    std::vector<T*>& detachData()
    {
        CV_Assert(cleanupMode == false);  // state is not valid
        std::vector<void*> dataVoid;
        {
            TLSDataContainer::detachData(dataVoid);
        }
        {
            AutoLock lock(mutex);
            detachedData.reserve(dataVoid.size() + dataFromTerminatedThreads.size());
            for (typename std::vector<T*>::const_iterator i = dataFromTerminatedThreads.begin(); i != dataFromTerminatedThreads.end(); ++i)
            {
                detachedData.push_back((T*)*i);
            }
            dataFromTerminatedThreads.clear();
            for (typename std::vector<void*>::const_iterator i = dataVoid.begin(); i != dataVoid.end(); ++i)
            {
                detachedData.push_back((T*)(void*)*i);
            }
        }
        dataVoid.clear();
        return detachedData;
    }

    /// Release associated thread data returned by detachData() call
    void cleanupDetachedData()
    {
        AutoLock lock(mutex);
        cleanupMode = true;
        _cleanupDetachedData();
        cleanupMode = false;
    }

    /// Release associated thread data
    void cleanup()
    {
        cleanupMode = true;
        TLSDataContainer::cleanup();

        AutoLock lock(mutex);
        _cleanupDetachedData();
        _cleanupTerminatedData();
        cleanupMode = false;
    }

    /// Release associated thread data and free TLS key
    void release()
    {
        cleanupMode = true;
        TLSDataContainer::release();
        {
            AutoLock lock(mutex);
            _cleanupDetachedData();
            _cleanupTerminatedData();
        }
    }

protected:
    // synchronized
    void _cleanupDetachedData()
    {
        for (typename std::vector<T*>::iterator i = detachedData.begin(); i != detachedData.end(); ++i)
        {
            deleteDataInstance((T*)*i);
        }
        detachedData.clear();
    }

    // synchronized
    void _cleanupTerminatedData()
    {
        for (typename std::vector<T*>::iterator i = dataFromTerminatedThreads.begin(); i != dataFromTerminatedThreads.end(); ++i)
        {
            deleteDataInstance((T*)*i);
        }
        dataFromTerminatedThreads.clear();
    }

protected:
    virtual void* createDataInstance() const CV_OVERRIDE
    {
        // Note: we can collect all allocated data here, but this would require raced mutex locks
        return new T;
    }
    virtual void  deleteDataInstance(void* pData) const CV_OVERRIDE
    {
        if (cleanupMode)
        {
            delete (T*)pData;
        }
        else
        {
            AutoLock lock(mutex);
            dataFromTerminatedThreads.push_back((T*)pData);
        }
    }
};
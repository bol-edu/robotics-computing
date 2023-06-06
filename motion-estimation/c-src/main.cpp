#include <iostream>
#include "EstimateMotion.h"
#include "Matrix.h"

using namespace std;

void getData(Matrix &match, Matrix &k, Matrix &kp0, Matrix &kp1, Matrix &depth)
{
    FILE *fpmatch;
    fpmatch = fopen("testdata\\match.txt", "r");
    int row;
    fscanf(fpmatch, "%d", &row);
    match = Matrix(row, 2);
    for (int i = 0; i < row; i++)
    {
        int m, n;
        fscanf(fpmatch, "%d %d", &m, &n);
        match.val[i][0] = m;
        match.val[i][1] = n;
    }
    fclose(fpmatch);

    FILE *fpk;
    fpk = fopen("testdata/k.txt", "r");
    k = Matrix(3, 3);
    for (int i = 0; i < 9; i++)
    {
        FLOAT m;
        fscanf(fpk, "%lf", &m);
        k.val[i / 3][i % 3] = m;
    }
    fclose(fpk);

    FILE *fpkp0;
    fpkp0 = fopen("testdata/kp0.txt", "r");
    fscanf(fpkp0, "%d", &row);
    kp0 = Matrix(row, 2);
    for (int i = 0; i < row; i++)
    {
        FLOAT m, n;
        fscanf(fpkp0, "%lf %lf", &m, &n);
        kp0.val[i][0] = m;
        kp0.val[i][1] = n;
    }
    fclose(fpkp0);

    FILE *fpkp1;
    fpkp1 = fopen("testdata/kp1.txt", "r");
    fscanf(fpkp1, "%d", &row);
    kp1 = Matrix(row, 2);
    for (int i = 0; i < row; i++)
    {
        FLOAT m, n;
        fscanf(fpkp1, "%lf %lf", &m, &n);
        kp1.val[i][0] = m;
        kp1.val[i][1] = n;
    }
    fclose(fpkp1);

    FILE *fpdepth;
    fpdepth = fopen("testdata/depth.txt", "r");
    int col;
    fscanf(fpdepth, "%d %d", &row, &col);
    depth = Matrix(row, col);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            FLOAT m;
            fscanf(fpdepth, "%lf", &m);
            depth.val[i][j] = m;
        }
    }
    fclose(fpdepth);
}

int main()
{
    Matrix match, k, kp0, kp1, depth;
    getData(match, k, kp0, kp1, depth);

    EstimateMotion em = EstimateMotion();
    Matrix rmat, tvec;
    em.estimate(match, kp0, kp1, k, depth, rmat, tvec);
    cout << "rmat\n"
         << rmat << endl
         << endl;
    cout << "tvec\n"
         << tvec << endl
         << endl;
}
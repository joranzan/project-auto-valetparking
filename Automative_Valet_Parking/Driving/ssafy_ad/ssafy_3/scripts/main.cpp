#include <stdio.h>
#include <iostream>
#include "dbscan.h"
#include <cmath>
#include <time.h>

#define MINIMUM_POINTS 20     // minimum number of cluster
#define EPSILON (0.75*0.75)  // distance for clustering, metre^2
#define MAXIMUM_DIST 1000

void readBenchmarkData(vector<Point>& points)
{
    // load point cloud
    FILE *stream;
    stream = fopen ("point_cloud2.txt","ra");

    unsigned int minpts, num_points, cluster, i = 0;
    double epsilon;
    fscanf(stream, "%u\n", &num_points);

    Point *p = (Point *)calloc(num_points, sizeof(Point));

    while (i < num_points)
    {
        fscanf(stream, "%f %f %f\n", &(p[i].x), &(p[i].y), &(p[i].z));
        float dist = sqrt((p[i].x*p[i].x)+(p[i].y*p[i].y)+(p[i].z*p[i].z));
        p[i].clusterID = UNCLASSIFIED;
        if (dist < MAXIMUM_DIST) {
            points.push_back(p[i]);
        }
          ++i;
    }

    free(p);
    fclose(stream);
}

void printResults(vector<Point>& points, int num_points)
{
    int arr[1000] = {0,};
    float coord[1000][3] = {0.0,};
    int i = 0;

    int max_clusterID = 0;
    
    printf("Number of points: %u\n"
        " x     y     z     cluster_id\n"
        "-----------------------------\n"
        , num_points);
    while (i < num_points)
    {
        // printf("%5.2lf %5.2lf %5.2lf: %d\n",
        //          points[i].x,
        //          points[i].y, points[i].z,
        //          points[i].clusterID);
        
        if (points[i].clusterID > max_clusterID) max_clusterID = points[i].clusterID;

        arr[points[i].clusterID]++;
        coord[points[i].clusterID][0] += points[i].x;
        coord[points[i].clusterID][1] += points[i].y;
        coord[points[i].clusterID][2] += points[i].z;
        ++i;
    }

    printf("max cluster ID: %d \n", max_clusterID);

    printf("Number of points: %u\n", num_points);
    printf("|Cluster ID | num points |  x  |  y  |  z  |\n");
    for (int j = 0; j <= max_clusterID; j++) {
        // if (arr[j] == 0) break;
        printf("|  %2d   |  %5d  |", j, arr[j]);
        for (int k = 0; k < 3; k++) {
            float coo = coord[j][k]/arr[j];
            printf("  %f  |", coo);
        }
        printf("\n");
    }
}

int main()
{    
    vector<Point> points;

    // read point data
    readBenchmarkData(points);
    
    printf("========================================\n");
    // constructor
    DBSCAN ds(MINIMUM_POINTS, EPSILON, points);
    printf("========================================\n");

    // main loop
    ds.run();

    printf("========================================\n");
    // // result of DBSCAN algorithm
    printResults(ds.m_points, ds.getTotalPointSize());    

    return 0;
}

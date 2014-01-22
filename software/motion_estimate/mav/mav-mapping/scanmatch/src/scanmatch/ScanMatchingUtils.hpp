/**
 * SECTION:scanmatch
 * @title: ScanMatchingUtils
 * @short_description: Various utility functions used in the scan matching routines
 * @include: bot/scanmatch/ScanMatchingUtils.hpp
 *
 * Various utility functions used in the scan matching routines
 *
 * many of these were stolen from elsewhere in carmen3D/agile, and placed here to make the scan matcher
 * somewhat standalone.
 *
 * Linking: -lscanmatch
 * namespace: scanmatch
 */

#ifndef SCANMATCHINGUTILS_
#define SCANMATCHINGUTILS_
#include <math.h>
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

namespace scanmatch
{

#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * basic 2D point structure used by the scan matcher... this should be interchangable with the point2d_t
 * structure used in agile etc...
 */
typedef struct
{
    double x;
    double y;
} smPoint;

#define smPoint_as_array(p) ((double*)p)

/* The magic below allows you to use the SMPOINT() macro to convert a
 * double[2] to a smPoint.  gcc is smart -- if you try to cast anything
 * other than a smPoint or a double[] with this macro, gcc will emit
 * a warning. */
union _smPoint_any_t
{
    smPoint point;
    double array[2];
};
#define SMPOINT(p) (&(((union _smPoint_any_t *)(p))->point))

/**
 * structure which stores a candidate scan transformation, along with some metadata.
 */
typedef struct
{
    double x;
    double y;
    double theta;
    double score;
    int hits;

    double sigma[3 * 3];

} ScanTransform;

/**
 * currently known/handled laser types
 */
typedef enum
{
    SM_HOKUYO_UTM, SM_HOKUYO_URG, SM_SICK_LMS, SM_DUMMY_LASER
} sm_laser_type_t;

static inline void
sm_vector_scale_3d(double v[3], double s)
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
}

static inline void
sm_vector_scale_nd(double * v, int N, double s)
{
    int i;
    for (i = 0; i < N; i++)
        v[i] *= s;
}

static inline void
sm_vector_vector_outer_product_3d(const double v1[3], const double v2[3],
        double result[9])
{
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[0];
    result[2] = v1[2] * v2[0];
    result[3] = result[1];
    result[4] = v1[1] * v2[1];
    result[5] = v1[2] * v2[1];
    result[6] = result[2];
    result[7] = result[5];
    result[8] = v1[2] * v2[2];
}

static inline void
sm_vector_add_3d(const double v1[3], const double v2[3], double result[3])
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

static inline void
sm_vector_add_2d(const double v1[2], const double v2[2], double result[2])
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
}

static inline void
sm_vector_add_nd(const double * v1, const double * v2, int N, double * result)
{
    int i;
    for (i = 0; i < N; i++)
        result[i] = v1[i] + v2[i];
}

static inline void
sm_vector_sub_3d(const double v1[3], const double v2[3], double result[3])
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

static inline void
sm_vector_sub_2d(const double v1[2], const double v2[2], double result[2])
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
}

static inline void
sm_vector_sub_nd(const double * v1, const double * v2, int N, double * result)
{
    int i;
    for (i = 0; i < N; i++)
        result[i] = v1[i] - v2[i];
}

static inline void
sm_matrix_multiply_3x3_3x3(const double a[9], const double b[9], double r[9])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double acc = 0;
            for (int k = 0; k < 3; k++)
                acc += a[3 * i + k] * b[j + 3 * k];
            r[i * 3 + j] = acc;
        }
    }
}

inline int
sm_clamp(int v, int min, int max)
{
    if (v > max)
        return max;
    if (v < min)
        return min;
    return v;
}

inline double
sm_sq(double v)
{
    return v * v;
}

inline double
sm_dist(const smPoint * p1, const smPoint * p2)
{
    return sqrt(sm_sq(p1->x - p2->x) + sm_sq(p1->y - p2->y));
}

inline double
sm_norm(const smPoint * p)
{
    return sqrt(sm_sq(p->x) + sm_sq(p->y));
}

inline double
sm_normalize_theta(double theta)
{
    int multiplier;

    if (theta >= -PI && theta < PI)
        return theta;

    multiplier = (int) (theta / (2 * PI));
    theta = theta - multiplier * 2 * PI;
    if (theta >= PI)
        theta -= 2 * PI;
    if (theta < -PI)
        theta += 2 * PI;

    return theta;
}

inline double
sm_angle_subtract(double theta1, double theta2)
{
    return sm_normalize_theta(sm_normalize_theta(theta1) - sm_normalize_theta(
            theta2));

}

inline void
sm_normalize_vector(smPoint * v)
{
    double n = sm_norm(v);
    v->x /= n;
    v->y /= n;
}

inline double
sm_dot(const smPoint * p1, const smPoint * p2)
{
    return p1->x * p2->x + p1->y * p2->y;
}

inline double
sm_angle_between_points(const smPoint * p1, const smPoint * p2, const smPoint * p3)
{
    smPoint v1 =
        { p2->x - p1->x, p2->y - p1->y };
    smPoint v2 =
        { p3->x - p2->x, p3->y - p2->y };
    double dot = sm_dot(&v1, &v2);
    return sm_normalize_theta(acos(dot / (sm_norm(&v1) * sm_norm(&v2))));
}

inline int
sm_imax(int v1, int v2)
{
    if (v1 > v2)
        return v1;
    else
        return v2;
}

inline int
sm_imin(int v1, int v2)
{
    if (v1 < v2)
        return v1;
    else
        return v2;
}

inline unsigned char
sm_ucmax(unsigned char v1, unsigned char v2)
{
    if (v1 > v2)
        return v1;
    else
        return v2;
}

inline char
sm_ucmin(unsigned char v1, unsigned char v2)
{
    if (v1 < v2)
        return v1;
    else
        return v2;
}

inline double
sm_fsign(double num)
{
    if (num > 0)
        return 1;
    else if (num == 0)
        return 0;
    else
        return -1;
}

inline void
sm_rotate2D(const double p[2], double theta, double rp[2])
{
    double c = cos(theta);
    double s = sin(theta);
    rp[0] = c * p[0] - s * p[1];
    rp[1] = s * p[0] + c * p[1];
}

inline void
sm_rotateCov2D(const double cov[9], double theta, double rCov[9])
{
    // compute R^-1*cov*R
    double c = cos(theta);
    double s = sin(theta);
    double R[9] =
        { c, -s, 0, s, c, 0, 0, 0, 1 };
    double Rinv[9] =
        { c, s, 0, -s, c, 0, 0, 0, 1 };
    double tmp[9]; //multiply doesn't work in place...
    sm_matrix_multiply_3x3_3x3(cov, R, tmp);
    sm_matrix_multiply_3x3_3x3(Rinv, tmp, rCov);
}

inline double
sm_dist_to_line(const smPoint * pt,const  smPoint * v1,const  smPoint * v2)
{ //compute distance to line that passes through these two points
    //expanded ugliness from symbolic matlab:
    //    a = v1 - v2;
    //    b = pt - v2;
    //    d = abs(det([a;b])) / sqrt(dot(a,a));
    return fabs(-v1->x * pt->y + v1->x * v2->y + v2->x * pt->y + v1->y * pt->x
            - v1->y * v2->x - v2->y * pt->x) / sqrt(v1->x * v1->x - 2 * v1->x
            * v2->x + v2->x * v2->x + v1->y * v1->y - 2 * v1->y * v2->y + v2->y
            * v2->y);
}

inline double
sm_dist_to_segment(const smPoint * pt, const smPoint * v1, const smPoint * v2, int * vertexNum=NULL)
{ //compute the distance to the line segment, or the closest endpoint if the point isn't inside
    double dot1 = (v2->x - v1->x) * (pt->x - v1->x) + (v2->y - v1->y) * (pt->y
            - v1->y);
    double dot2 = (v1->x - v2->x) * (pt->x - v2->x) + (v1->y - v2->y) * (pt->y
            - v2->y);
    int insideSegment = dot1 > 0 && dot2 > 0;
    if (insideSegment) //we're inside the segment, so return the perpendicular dist
    {
      if (vertexNum)
        *vertexNum = 0;
        return sm_dist_to_line(pt, v1, v2);
    } else {
        double dv1 = sm_dist(pt, v1);
        double dv2 = sm_dist(pt, v2);
        if (dv1 < dv2) {
          if (vertexNum)
            *vertexNum = 1;
            return dv1;
        } else {
          if (vertexNum)
            *vertexNum = 2;
            return dv2;
        }
    }

}

inline void
sm_transformPoints(const ScanTransform *T, const smPoint * points, unsigned numPoints,
        smPoint * ppoints)
{
    double ct = cos(T->theta), st = sin(T->theta);
    for (unsigned i = 0; i < numPoints; i++) {
        ppoints[i].x = T->x + ct * points[i].x - st * points[i].y;
        ppoints[i].y = T->y + st * points[i].x + ct * points[i].y;
    }
}

inline int
sm_projectRangesToPoints(const float * ranges, int numPoints, double thetaStart,
        double thetaStep, smPoint * points, double maxRange = 1e10,
        double validRangeStart = -1000, double validRangeEnd = 1000,
        double * aveValidRange = NULL, double * stddevValidRange = NULL)
{
    int count = 0;
    double aveRange = 0;
    double aveRangeSq = 0;

    double theta = thetaStart;
    for (int i = 0; i < numPoints; i++) {
        if (ranges[i] > .1 && ranges[i] < maxRange && theta > validRangeStart
                && theta < validRangeEnd) { //hokuyo driver seems to report maxRanges as .001 :-/
            //project to body centered coordinates
            points[count].x = ranges[i] * cos(theta);
            points[count].y = ranges[i] * sin(theta);
            count++;
            aveRange += ranges[i];
            aveRangeSq += sm_sq(ranges[i]);
        }

        theta += thetaStep;
    }

    aveRange /= (double) count;
    aveRangeSq /= (double) count;

    if (aveValidRange != NULL)
        *aveValidRange = aveRange;
    if (stddevValidRange != NULL)
        *stddevValidRange = sqrt(aveRangeSq - sm_sq(aveRange));

    return count;
}

inline int
sm_projectRangesAndDecimate(int beamskip, float spatialDecimationThresh,
    const float * ranges, int numPoints, double thetaStart, double thetaStep,
        smPoint * points, double maxRange = 1e10, double validRangeStart =
                -1000, double validRangeEnd = 1000)
{
    int lastAdd = -1000;
    double aveRange;
    double stdDevRange;
    int numValidPoints = sm_projectRangesToPoints(ranges, numPoints,
            thetaStart, thetaStep, points, maxRange, validRangeStart,
            validRangeEnd, &aveRange, &stdDevRange);

    smPoint origin =
        { 0, 0 };
    smPoint lastAddPoint =
        { 0, 0 };
    int numDecPoints = 0;
    for (int i = 0; i < numValidPoints; i++) {
        //add every beamSkip beam, unless points are more than spatialDecimationThresh, or more than 1.8 stdDevs more than ave range
        if ((i - lastAdd) > beamskip || sm_dist(&points[i], &lastAddPoint)
                > spatialDecimationThresh || sm_dist(&points[i], &origin)
                > (aveRange + 1.8 * stdDevRange)) {
            lastAdd = i;
            lastAddPoint = points[i];
            points[numDecPoints] = points[i]; // ok since i >= numDecPoints
            numDecPoints++;
        }
    }
    return numDecPoints;
}

inline double
sm_get_time(void)
{
    struct timeval tv;
    double t;

    if (gettimeofday(&tv, NULL) < 0)
        fprintf(stderr, "sm_get_time encountered error in gettimeofday\n");
    t = tv.tv_sec + tv.tv_usec / 1000000.0;
    return t;
}

inline int64_t
sm_get_utime(void)
{
    struct timeval tv;
    int64_t ut;

    if (gettimeofday(&tv, NULL) < 0)
        fprintf(stderr, "sm_get_time encountered error in gettimeofday\n");
    ut = tv.tv_sec * 1000000.0 + tv.tv_usec;
    return ut;
}

static inline sm_laser_type_t
sm_get_laser_type_from_name(const char * laser_name)
{
    if (strcmp(laser_name, "HOKUYO_UTM") == 0) {
        return SM_HOKUYO_UTM;
    } else if (strcmp(laser_name, "HOKUYO_URG") == 0) {
        return SM_HOKUYO_URG;
    } else if (strcmp(laser_name, "SICK_LMS") == 0) {
        return SM_SICK_LMS;
    } else
        return SM_DUMMY_LASER;
}

/** Given an array of colors, a palette is created that linearly interpolates through all the colors. **/
static void
sm_color_util_build_color_table(double color_palette[][3], int palette_size,
        float lut[][3], int lut_size)
{
    for (int idx = 0; idx < lut_size; idx++) {
        double znorm = ((double) idx) / lut_size;

        int color_index = (palette_size - 1) * znorm;
        double alpha = (palette_size - 1) * znorm - color_index;

        for (int i = 0; i < 3; i++) {
            lut[idx][i] = color_palette[color_index][i] * (1.0 - alpha)
                    + color_palette[color_index + 1][i] * alpha;
        }
    }
}

#define SM_JET_COLORS_LUT_SIZE 1024
static float sm_jet_colors[SM_JET_COLORS_LUT_SIZE][3];
static int sm_jet_colors_initialized = 0;

static void
sm_init_color_table_jet()
{
    double jet[][3] =
        {
            { 0, 0, 1 },
            { 0, .5, .5 },
            { .8, .8, 0 },
            { 1, 0, 0 } };

    sm_color_util_build_color_table(jet, sizeof(jet) / (sizeof(double) * 3),
            sm_jet_colors, SM_JET_COLORS_LUT_SIZE);
    sm_jet_colors_initialized = 1;
}

static inline float *
sm_color_util_jet(double v)
{
    if (!sm_jet_colors_initialized)
        sm_init_color_table_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (SM_JET_COLORS_LUT_SIZE - 1) * v;
    return sm_jet_colors[idx];
}

// Returns the hash value of the null terminated C string 'string' using the
// SDBM hash algorithm. The number of significant characters for which the
// hash value will be calculated is limited to MAXIMUM_LENGTH_FOR_STRINGS.
inline unsigned int
sm_hash(const char *string)
{
    register unsigned int len, index, hash = 0;
    register char ch;

    len = strlen(string);
    if (len > 1000) {
        len = 1000;
    } // end if

    for (index = 0; index < len; index++) {
        ch = string[index];
        hash = ch + (hash << 6) + (hash << 16) - hash;
    } // end for

    return (hash & 0x7FFFFFFF);
} // end calc_hash

//#define DISABLE_TICTOC
//#define PRINT_TO_FILE
static pthread_mutex_t tictoc_lock;
static int tictoc_initialized = 0;
inline void
sm_tictoc_init() //this MUST be called if you want tictoc to be thread safe
{
#ifdef DISABLE_TICTOC
    return;
#endif
    pthread_mutex_init(&tictoc_lock, NULL);
    tictoc_initialized = 1;

}

inline double
sm_tictoc(const char *description)
{

#ifdef DISABLE_TICTOC
    return -1;
#endif
    //profiling tool... call the first time to set clock going, call again to stop clock.
    //call with NULL to print results
    //need to call init before use to setup locks if you want it to be threadSafe...

    double timeTaken = -1;

    if (tictoc_initialized)
        pthread_mutex_lock(&tictoc_lock); //aquire the lock

    static double t[1000] =
        { 0 };
    static double totalT[1000] =
        { 0 };
    static int numCalls[1000] =
        { 0 };
    static char flag[1000] =
        { 0 };
    static char * descriptions[1000];
    if (description != NULL) {

        unsigned idx = sm_hash(description) % 1000;
        if (flag[idx] == 0) {
            //printf("tic \n");
            t[idx] = sm_get_time();
            flag[idx] = 1;
        } else if (flag[idx] == 1) {
            timeTaken = sm_get_time() - t[idx];
            totalT[idx] = totalT[idx] + timeTaken;
            if (numCalls[idx] == 0) {
                //first time... store description
                descriptions[idx] = (char *) malloc((strlen(description) + 1)
                        * sizeof(char));
                strcpy(descriptions[idx], description);
            }
            if (strcmp(descriptions[idx], description) != 0) {
                printf(
                        "ERROR! HASH COLLISION in tictoc, choose a different description!!!\n");
                printf("%s collided with %s, both had has=%d\n",
                        descriptions[idx], description, idx);
                exit(1);
            }
            numCalls[idx]++;

            //printf("toc %d \n",numCalls[idx]);
            flag[idx] = 0;
        }
    } else {
#ifdef PRINT_TO_FILE
        FILE * f = fopen("tictoc.txt", "w");
#endif
        printf("\n\n\n");
        printf("timer results (alphabetical):\n\n");
        int i, j;
        int lowestInd;
        char lowest[256];
        char spaces[256];
        char flag[1000];
        for (i = 0; i < 1000; i++)
            flag[i] = 0;
        for (i = 0; i < 1000; i++) {
            strcpy(lowest, "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
            lowestInd = -1;
            for (j = 0; j < 1000; j++) {
                //find first one in alphabetical order
                if (numCalls[j] > 0 && flag[j] == 0) {
                    if (strcmp(descriptions[j], lowest) < 0) {
                        lowestInd = j;
                        strcpy(lowest, descriptions[j]);
                    }
                }
            }
            if (lowestInd == -1)
                break;
            else {
                int numSpaces = 50 - strlen(descriptions[lowestInd]);
                for (j = 0; j < numSpaces; j++)
                    spaces[j] = ' ';
                spaces[j] = '\0';
                printf(
                        "%s %s\t called %d times,\t took %f Average, \t %f total\n",
                        descriptions[lowestInd], spaces, numCalls[lowestInd],
                        totalT[lowestInd] / numCalls[lowestInd],
                        totalT[lowestInd]);
#ifdef PRINT_TO_FILE
                fprintf(f, "%s %s\t called %d times,\t took %f Average, \t %f total\n", descriptions[lowestInd], spaces,
                        numCalls[lowestInd], totalT[lowestInd] / numCalls[lowestInd], totalT[lowestInd]);
#endif
                flag[lowestInd] = 1;
            }
        }

        printf("\n\n\n");
        printf("timer results (by average time):\n\n");
        int maxInd;
        double maxTime;

        for (i = 0; i < 1000; i++)
            flag[i] = 0;

        for (i = 0; i < 1000; i++) {
            maxTime = -1;
            maxInd = -1;
            for (j = 0; j < 1000; j++) {
                //find first one in alphabetical order
                if (numCalls[j] > 0 && flag[j] == 0) {
                    if (totalT[j] / numCalls[j] > maxTime) {
                        maxInd = j;
                        maxTime = totalT[j] / numCalls[j];
                    }
                }
            }
            if (maxInd == -1)
                break;
            else {
                int numSpaces = 50 - strlen(descriptions[maxInd]);
                for (j = 0; j < numSpaces; j++)
                    spaces[j] = ' ';
                spaces[j] = '\0';
                printf(
                        "%s %s\t called %d times,\t took %f Average, \t %f total\n",
                        descriptions[maxInd], spaces, numCalls[maxInd],
                        totalT[maxInd] / numCalls[maxInd], totalT[maxInd]);
#ifdef PRINT_TO_FILE
                fprintf(f, "%s %s\t called %d times,\t took %f Average, \t %f total\n", descriptions[maxInd], spaces,
                        numCalls[maxInd], totalT[maxInd] / numCalls[maxInd], totalT[maxInd]);
#endif
                flag[maxInd] = 1;
            }
        }

        printf("\n\n\n");
        printf("timer results (by total time):\n\n");

        for (i = 0; i < 1000; i++)
            flag[i] = 0;

        for (i = 0; i < 1000; i++) {
            maxTime = -1;
            maxInd = -1;
            for (j = 0; j < 1000; j++) {
                //find first one in alphabetical order
                if (numCalls[j] > 0 && flag[j] == 0) {
                    if (totalT[j] > maxTime) {
                        maxInd = j;
                        maxTime = totalT[j];
                    }
                }
            }
            if (maxInd == -1)
                break;
            else {
                int numSpaces = 50 - strlen(descriptions[maxInd]);
                for (j = 0; j < numSpaces; j++)
                    spaces[j] = ' ';
                spaces[j] = '\0';
                printf(
                        "%s %s\t called %d times,\t took %f Average, \t %f total\n",
                        descriptions[maxInd], spaces, numCalls[maxInd],
                        totalT[maxInd] / numCalls[maxInd], totalT[maxInd]);
#ifdef PRINT_TO_FILE
                fprintf(f, "%s %s\t called %d times,\t took %f Average, \t %f total\n", descriptions[maxInd], spaces,
                        numCalls[maxInd], totalT[maxInd] / numCalls[maxInd], totalT[maxInd]);
#endif
                flag[maxInd] = 1;
            }
        }
#ifdef PRINT_TO_FILE
        fclose(f);
#endif
    }
    if (tictoc_initialized)
        pthread_mutex_unlock(&tictoc_lock);
    return timeTaken;
}

/*
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 */

#define ELEM_SWAP(a,b) { register float t=(a);(a)=(b);(b)=t; }

inline float
sm_median(float arr[], int n)
{
    int low, high;
    int median;
    int middle, ll, hh;

    low = 0;
    high = n - 1;
    median = (low + high) / 2;
    for (;;) {
        if (high <= low) /* One element only */
            return arr[median];

        if (high == low + 1) { /* Two elements only */
            if (arr[low] > arr[high])
                ELEM_SWAP(arr[low], arr[high]);
            return arr[median];
        }

        /* Find median of low, middle and high items; swap into position low */
        middle = (low + high) / 2;
        if (arr[middle] > arr[high])
            ELEM_SWAP(arr[middle], arr[high]);
        if (arr[low] > arr[high])
            ELEM_SWAP(arr[low], arr[high]);
        if (arr[middle] > arr[low])
            ELEM_SWAP(arr[middle], arr[low]);

        /* Swap low item (now in position middle) into position (low+1) */
        ELEM_SWAP(arr[middle], arr[low+1]);

        /* Nibble from each end towards middle, swapping items when stuck */
        ll = low + 1;
        hh = high;
        for (;;) {
            do
                ll++;
            while (arr[low] > arr[ll]);
            do
                hh--;
            while (arr[hh] > arr[low]);

            if (hh < ll)
                break;

            ELEM_SWAP(arr[ll], arr[hh]);
        }

        /* Swap middle item (in position low) back into correct position */
        ELEM_SWAP(arr[low], arr[hh]);

        /* Re-set active partition */
        if (hh <= median)
            low = ll;
        if (hh >= median)
            high = hh - 1;
    }
}

#undef ELEM_SWAP

}
#endif /*SCANMATCHINGUTILS_*/


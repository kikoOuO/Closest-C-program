#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <math.h>

#include "point.h"
#include "serial_closest.h"
#include "parallel_closest.h"
#include "utilities_closest.h"


/*
 * Multi-process (parallel) implementation of the recursive divide-and-conquer
 * algorithm to find the minimal distance between any two pair of points in p[].
 * Assumes that the array p[] is sorted according to x coordinate.
 */

// All the comments written down below are made by Jingbo Yang in use of clarification
double closest_parallel(struct Point *p, int n, int pdmax, int *pcount) {
    if (n < 4 || pdmax == 0) {
        return closest_serial(p, n);
    }

    // Split the graph into halve
    int mid = n / 2;
    struct Point *left = p;
    struct Point *right = p + mid;
    int left_n = mid;
    int right_n = n - mid;

    //Create pipe connection
    int pipe_left[2], pipe_right[2];
    if (pipe(pipe_left) == -1 || pipe(pipe_right) == -1) {
        perror("pipe failed");
        exit(1);
    }

    //Process pipe_left, calculate the closest distance in left subarea recursively
    //according to the value of pdmax
    pid_t left_pid = fork();
    if (left_pid < 0) {
        perror("fork failed");
        exit(1);
    } else if (left_pid == 0) {
        close(pipe_left[0]);
        double left_distance = closest_parallel(left, left_n, pdmax - 1, pcount);
        write(pipe_left[1], &left_distance, sizeof(left_distance));
        close(pipe_left[1]);
        exit(*pcount);
    }

    // Same thing happens for right subarea
    pid_t right_pid = fork();
    if (right_pid < 0) {
        perror("fork failed");
        exit(1);
    } else if (right_pid == 0) {
        close(pipe_right[0]);
        double right_distance = closest_parallel(right, right_n, pdmax - 1, pcount);
        write(pipe_right[1], &right_distance, sizeof(right_distance));
        close(pipe_right[1]);
        exit(*pcount);
    }

    close(pipe_left[1]);
    close(pipe_right[1]);

    int status_left, status_right;
    waitpid(left_pid, &status_left, 0);
    waitpid(right_pid, &status_right, 0);

    //extract information out from pipe and store in left and right distance
    double left_distance, right_distance;
    read(pipe_left[0], &left_distance, sizeof(left_distance));
    read(pipe_right[0], &right_distance, sizeof(right_distance));
    close(pipe_left[0]);
    close(pipe_right[0]);

    // main calculation logic
    *pcount += WEXITSTATUS(status_left) + WEXITSTATUS(status_right) + 2;
    double strip_distance = closest_serial(p, n);
    return fmin(fmin(left_distance, right_distance), strip_distance);
}


#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include <AprilTags/apriltag.h>
#include <AprilTags/common/image_u8.h>

#include <AprilTags/tag36h11.h>
#include <AprilTags/tag36h10.h>
#include <AprilTags/tag36artoolkit.h>
#include <AprilTags/tag25h9.h>
#include <AprilTags/tag25h7.h>

#include <AprilTags/common/zarray.h>
#include <AprilTags/common/getopt.h>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);
    apriltag_family_t *tf = tag36h11_create();

    tf->black_border = getopt_get_int(getopt, "border");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    int quiet = getopt_get_bool(getopt, "quiet");


    const int hamm_hist_max = 10;

    for (int input = 0; input < zarray_size(inputs); input++) {

        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));

        char *path;
        zarray_get(inputs, input, &path);
        if (!quiet)
            printf("loading %s\n", path);

        image_u8_t *im = image_u8_create_from_pnm(path);
        if (im == NULL) {
            printf("couldn't find %s\n", path);
            continue;
        }

        zarray_t *detections = apriltag_detector_detect(td, im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (!quiet)
                printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
                       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);

            hamm_hist[det->hamming]++;
        }

        apriltag_detections_destroy(detections);

        if (!quiet) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
        }

        if (!quiet)
            printf("Hamming histogram: ");

        for (int i = 0; i < hamm_hist_max; i++)
            printf("%5d", hamm_hist[i]);

        if (quiet) {
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
        }

        printf("\n");

        image_u8_destroy(im);
    }
    

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf);
    return 0;
}

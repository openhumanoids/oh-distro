#!/bin/bash
#

if [ $# -ne 2 ]; then
    echo "Usage: `basename $0` ppms_directory output_file_name"
    exit
fi

video_corpus_dir=$1
video_file_name=$2

num_cpus=`grep processor /proc/cpuinfo | wc -l`

if [ -d ${video_corpus_dir} ]; then
    echo "Processing image files in $video_corpus_dir"
else
    echo "Video directory: $video_corpus_dir doesn't exist. Exiting"
    exit 1
fi

rm -f {video_corpus_dir}/*.png

echo "Converting videos to png"
count=0
for img in ${video_corpus_dir}/*.ppm; do
    mogrify -format png $img &
    let count+=1
    [[ $((count%num_cpus)) -eq 0 ]] && wait
done

#mogrify -format jpg -quality 95 ${video_corpus_dir}/*.ppm
#rm -f ${video_corpus_dir}/*.ppm

echo "Creating movie ${video_file_name}"
#mencoder "mf://${video_corpus_dir}/*.png" -mf fps=30 -o ${video_file_name} -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=4096
mencoder "mf://${video_corpus_dir}/*.png" -mf fps=30 -o ${video_file_name} -ovc lavc -lavcopts vcodec=fmp4::vbitrate=4096

rm -f ${video_corpus_dir}/*.ppm
rm -f ${video_corpus_dir}/*.png

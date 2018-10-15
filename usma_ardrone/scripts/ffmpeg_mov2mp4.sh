MOV_IN=$1
MP4_OUT=$2

ffmpeg -i $MOV_IN.mov -vcodec copy -acodec copy $MP4_OUT.mp4

MOV_IN=$1

ffmpeg -i $MOV_IN.mov -vcodec copy -acodec copy $MOV_IN.mp4

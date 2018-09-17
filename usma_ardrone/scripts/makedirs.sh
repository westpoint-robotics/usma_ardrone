# DATE="20151001"
# call : $ . makedirs.sh 20160930 1 5

FOLDER="/home/$USER/ros/data"

DATE=$1
NEWDATE="$FOLDER/$DATE"
echo " "
echo " "
echo " "
echo $NEWDATE
mkdir -p $NEWDATE/csv
mkdir -p $NEWDATE/videos

echo " "


i=$2
STOP=$3

echo "i = $2"
echo "Stop = $3"
while [ $i -le $STOP ]
do
	RUN="$(printf "%03d" $i)"
	NEWRUNDIR="$FOLDER/$DATE/$RUN"

	echo $NEWRUNDIR
	mkdir $NEWRUNDIR

	i=$((i+=1))
done

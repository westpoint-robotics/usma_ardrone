#!/bin/sh

n=1
m=2
MASTERn=0
MASTERLIMIT=3
INSTALL_PATH=/home/default/ARAutoConnect
LOG=$INSTALL_PATH/logbox


# LAST_NUMBER="1 2 3 4 5 6 7 8"


# for i in $LAST_NUMBER
# do
#     if [ "$MASTERn" -ge "$MASTERLIMIT" ] ; then
#         echo "$MASTERn >= $MASTERLIMIT" >> $LOG
#     fi
#     MASTERn=`expr "$MASTERn" + 1`

# done



CONFIG=`iwconfig ath0`


echo $CONFIG

if echo $CONFIG | grep -q "Master" ;
then
    echo "$MASTERn >= $MASTERLIMIT" >> $LOG
fi



# if [ "$MASTERn" -le 1 ] ; then
#     echo "n <= 1"
# fi

# MASTERn=`expr "$MASTERn" + 1`

# if [ "$MASTERn" -eq 1 ] ; then
#     echo "n = 1"
# fi

# MASTERn=`expr "$MASTERn" + 1`

# if [ "$MASTERn" -ge 3 ] ; then
#     echo "n >= 1"
# fi



# # if [1 -lt 2] && echo "true" || echo "false"
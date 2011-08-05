#!/bin/bash

RELEASE=0.1-2011.08.05
nice debuild -S
sudo nice pbuilder build ../../libwou_${RELEASE}.dsc
sudo cp -v \
  /var/cache/pbuilder/result/libwou_${RELEASE}_i386.deb \
  /var/cache/pbuilder/result/libwou-dev_${RELEASE}_i386.deb \
  /var/cache/archive/lucid
sudo pbuilder --update --override-config
cp -v \
  /var/cache/pbuilder/result/libwou_${RELEASE}_i386.deb \
  ~/tmp/

#!/usr/bin/env bash

SRC="../564_proj"
if [ -e MyDesign.v ]; then
  rm MyDesign.v
fi
cat $SRC/MyDesign.v $SRC/sram_push.v $SRC/sram_pull.v $SRC/m_control.v $SRC/m_datapath.v $SRC/compression_datapath.v $SRC/compression_control.v $SRC/big_control.v $SRC/w_control.v $SRC/w_datapath.v > MyDesign.v 


#!/bin/bash
source='report'

echo "quickbuilding  "$source
pdflatex $source.tex
echo "Success opening Document "$source
evince $source.pdf &


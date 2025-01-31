#!/bin/bash

set -e

cd src/robot_test/data_analysis/
if [ ! -d venv ]
then
	python3 -m venv venv
fi

source venv/bin/activate
pip install fpdf \
	    matplotlib \
	    pandas

mkdir logs
exec python3 scripts/generate_report.py 

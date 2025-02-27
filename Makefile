.PHONY: all
all: build

.PHONY: build clean cleanall generate_report launch rqt
build clean cleanall generate_report launch rqt:
	bash $@.sh

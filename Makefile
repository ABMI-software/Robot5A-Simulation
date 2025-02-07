.PHONY: all
all: build

.PHONY: build clean generate_report launch rqt
build clean generate_report launch rqt:
	bash $@.sh

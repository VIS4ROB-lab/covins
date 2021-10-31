all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build docker image"
	@echo "   1. make clean              - remove all images"
	@echo ""

build:
	@echo "Use -j$(NR_JOBS) for build commands"
	@docker build --tag covins -f ./Dockerfile --build-arg NR_JOBS=$(NR_JOBS) ..

clean:
	@docker rmi -f covins

## Dockerized Make for Covins
SHELL := /usr/bin/env bash
NR_JOBS ?= $$(($$(nproc)/2))
# NR_JOBS ?= $(shell nproc)

# registry can be docker.io, ghcr.io, gcr.io for exmaple
REGISTRY ?= docker.io
ORG ?= vis4rob
IMAGE ?= covins
DOCKERFILE ?= Dockerfile
# we are building the parent
BUILD_PATH ?= ..
# use Dockerfile.dockerignore for files like README that should not break the
# cache
# https://github.com/docker/buildx/issues/166
# the --load not compatible with multiarch images so do not
# use with --platform=linux/arm64,linux/amd64 for example
OUTPUT ?= $(if $(PUSH),--push,--load)

# default tag is just the sha of the current git commit
# overwrite if you want with the specific tag in
# for instance make build TAG=v1.0.1
TAG ?= $(shell git rev-parse HEAD)
# Cannot run multiple arch if memory <= 7GB
PLATFORM ?= linux/amd64,linux/arm64
#PLATFORM ?= linux/amd64
#PLATFORM ?= linux/arm64

MEMORY != docker info | grep "Total Memory" | sed 's/[^0-9\.]*//g'

.DEFAULT_GOAL := help
.PHONY: help
# https://swcarpentry.github.io/make-novice/08-self-doc/ is simpler just need
help: $(MAKEFILE_LIST)
	@sed -n 's/^##//p' $(MAKEFILE_LIST)


## buildx: Use BuildKit if you want multiarch images
# create a builder with large log file
# latest is automatically applied so do not need to tag
.PHONY: buildx
buildx: warning
	@time ( \
		docker buildx build -t $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
			--platform $(PLATFORM) \
			--build-arg NR_JOBS=$(NR_JOBS) \
			-f $(DOCKERFILE) \
			$(OUTPUT) \
			$(BUILD_PATH) && \
		docker tag $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
			$(REGISTRY)/$(ORG)/$(IMAGE):latest \
		)

## push: push local images
.PHONY: push
push: 
		docker push --all-tags $(REGISTRY)/$(ORG)/$(IMAGE)

## build: build one architecture at time to lower memory used
.PHONY: build
build: warning
	@time ( \
		docker build -t $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
			--build-arg NR_JOBS=$(NR_JOBS) \
			-f $(DOCKERFILE) \
			$(BUILD_PATH) && \
		docker tag $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
			$(REGISTRY)/$(ORG)/$(IMAGE):latest \
	)

## debug: dump all docker buildx messages out
# create a temporary docker context with larger log size
# https://stackoverflow.com/questions/65819424/is-there-a-way-to-increase-the-log-size-in-docker-when-building-a-container
.PHONY: debug
debug:
	@time ( \
		if ! docker buildx ls | grep -q build-$(IMAGE); then \
			docker buildx create --name build-$(IMAGE) --use \
				--driver-opt env.BUILDKIT_STEP_LOG_MAX_SIZE=100000000 \
		; fi && \
		docker buildx use big && \
		docker buildx build -t $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
				$(OUTPUT) \
				--progress plain  \
				--platform $(PLATFORM) \
				--build-arg NR_JOBS=$(NR_JOBS) \
				-f $(DOCKERFILE) \
				$(BUILD_PATH) && \
		docker tag $(REGISTRY)/$(ORG)/$(IMAGE):$(TAG) \
			$(REGISTRY)/$(ORG)/$(IMAGE):latest && \
		docker buildx rm build-$(IMAGE) \
	)	

## tag: Add a github tag you can use with make build TAG=_your tag_
.PHONY: tag
tag:
	@git tag -a $(TAG) -m "tagged with $(TAG)" && \
	git push --tags

## warning: RAM and processor requirements
# note need to use bc because bash does not support floating comparison
# https://stackoverflow.com/questions/33111191/bash-bc-curl-comparing-variables
# Note that the values in a GiB so slightly more than a GB
.PHONY: warning
warning:
	@echo "$(MEMORY) RAM available need at least 7GB and 32GB for two architectures" && \
	if [[ $(PLATFORM) =~ , ]] && (( $$(bc <<<"$(MEMORY) <= 31") )); then \
		echo "WARNING: Multiple build require at least 32GB of RAM"; \
	fi && \
	if (( $$(bc <<<"$(MEMORY) <= 6.8") )); then \
	    echo "WARNING: build requires more than 7GB of RAM"; \
	fi

## stats: memory and other stats of containers
.PHONY: stats
stats:
	@docker stats

## clean: remove docker images
.PHONY: clean
clean:
	@docker rmi -f $(REGIStRY)/$(ORG)/$(IMAGE)

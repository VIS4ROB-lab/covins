# Dockerized COVINS build

In the original documentation, the advice is to set `make build NR_JOBS=14` and
when that fails keep decreasing it.

## Building Quickstart

You can now add a make parameter and the image registry, registry organization
and image name by setting. By default you build a single image to the local
docker cache:

```sh
# builds a single image
make build
```

## Pushing

If you want to push it you can either specify this at build time or you can do
it explicitlyi


```sh
# push the local image that was previously
make push
# this pushs by default to docker.io/vis4rbo/covins:v1.0.1
make build PUSH=1
# pushes to ghcr.io/netdrones/covins-build:v1.0.1
make build PUSH=1 REGISTRY=ghcr.io ORG=netdrones IMAGE=covins-build TAG=v1.0.1-netdrones
```

## Multiple architecture in a docker image

In the original system, only Intel images are created, but by changing the
PLATFORM environment variable, you can build for instance for both Intel and
Apple Silicon:

```sh
# build just you local machine architecture as noted in uname -m
make build
# build for intel and Apple silicon not you must have a push location
make buildx PLATFORM=linux/amd64,linux/arm64
```

### Number of jobs per processor cores and jobs per make

When you start docker, you can assign processor cores that it can use. The
build benefits from the concurrency of jobs, but at the base and run time
change significantly based on how much concurrency there. And you can also set
how many jobs per make by setting NR_JOBS.

Note that if you are building multiple images, these builds actually run
concurrently, so if you set NR_JOBS=5 with 5 cores then 5 jobs run for each
image. So if you are building two images, then you are actually scheduling
2*5 or 10 jobs in docker.

The default assumes you devote half the number of physical cores on your
machine with the utility `nproc` and so you are assigning one job per core.

```sh
# set to run with 10 jobs in the build
make build NR_JOBS=10
# this also sets 10 jobs because platform builds run concurrently
make build PLATFORM=linux/amd64,linux/arm64 NR_JOBS=5
```

## How much memory to allocate to docker: 7GB for single, 24GB for dual

The error that occurs is not easy to understand. It is a C++ compiler error
failing on number of variables being optimized, but it appears to be an out of
memory problem during the catkins process. Some experimentation is needed to
understand what the right build parameters on your configuration.

The original system only created the container for local use. You can still use
it this way, but given the long build times for the system on the order of
hours for typical laptops, here are some parameters for how much space to
allocate to the docker machine running. On a Docker Desktop for the Mac, this
is done with the graphical interface. Other systems such as colima or multipass
handle this with command line parameters. But here are some rough guidelines
with all tests done with the default GitHub Actions runner and an Apple MacBook
Pro (2021) with an M1 Max 10-core and 64GB memory:

- Build of single image succeeds using 7GB and 1 job for 2 cores after 2 hours.
  This is what can be used with GitHub Actions as that is the
- Build of dual Intel and Apple Silicon image fails with a gcc error using 8GB
  for NR_JOBS set to 5 jobs using 5 cores after 3 hours
- Build of dual fails with gcc error using 16GB and 10 jobs for 5 cores after 3 hours
- Build of dual gcc error with 24GB and 5 jobs for 5 cores after 2 hours
- Build of dual images succeeds with 32GB of RAM and 5 jobs/image for 5 cores
  after 2 hours of building
- Build of dual succeeds with 32GB and 10 jobs for 5 cores after 2 hours

## Debugging memory issues with docker stats

If you run in a separate window `make stats` then docker stats will run and you
can see the dynamic memory requirements, these tend to peak in the later phases
of the catkins build of covins.

SHELL:=/bin/bash

.DEFAULT_GOAL := all

.PHONY: all 
all: root_check docker_group_check build

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

CPP_PROJECT_DIRECTORY:="${ROOT_DIR}/adore_if_ros"

include adore_if_ros.mk 
include ${ADORE_IF_ROS_SUBMODULES_PATH}/ci_teststand/ci_teststand.mk

#CMAKE_PREFIX_PATH?=$(shell realpath "$$(find . -name install | grep "install" | grep -e "install/\|CPack" -v)" | tr '\n' ';') 
CMAKE_PREFIX_PATH?=$(shell realpath "$$(find . -type d | grep "build" | grep -v "build/")" 2>/dev/null | tr '\n' ';')

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_ROS_PROJECT}) 
	$(eval TAG := ${ADORE_IF_ROS_TAG})

.PHONY: _build
_build:
	docker build --tag ${PROJECT}:${TAG} \
                 --network host \
                 --build-arg ADORE_IF_ROS_MSG_TAG="${ADORE_IF_ROS_MSG_TAG}" \
                 --build-arg LIBADORE_TAG="${LIBADORE_TAG}" \
                 --build-arg ADORE_IF_ROS_SCHEDULING_TAG=${ADORE_IF_ROS_SCHEDULING_TAG} \
                 --build-arg PLOTLABLIB_TAG="${PLOTLABLIB_TAG}" .

.PHONY: build 
build: clean set_env build_adore_scheduling build_libadore build_plotlablib build_adore_if_ros_msg _build  docker_cp ## Build adore_if_ros

.PHONY: docker_cp
docker_cp: set_env
	rm -rf "${PROJECT}/build"
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build ${PROJECT}

.PHONY: clean_submodules
clean_submodules: clean_plotlablib clean_adore_if_ros_msg clean_adore_scheduling clean_libadore

.PHONY: clean 
clean: set_env clean_submodules
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	find . -name "**lint_report.log" -exec rm -rf {} \;
	find . -name "**cppcheck_report.log" -exec rm -rf {} \;
	find . -name "**lizard_report.xml" -exec rm -rf {} \;
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
	docker rmi $$(docker images --filter "dangling=true" -q) --force > /dev/null 2>&1 || true

.PHONY: static_checks
static_checks: lint lizard cppcheck

.PHONY: test
test: ci_test

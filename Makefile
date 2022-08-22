# Common vars 
PARENT_IMAGE_NAME=misterkoz/ros-melodic-with-vnc-and-gazebo
IMAGE_NAME=misterkoz/ros-melodic-with-vnc-and-gazebo_project

##A make file with docs!
##--------------------------------------------------------
help:              ## Show this help.
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

build:             ## Build the image
	docker build -t ${IMAGE_NAME} .

push:              ## Push the image
	docker login
	docker push ${IMAGE_NAME}

build-parent:      ## Build the image
	docker build -t ${PARENT_IMAGE_NAME} -f parent.Dockerfile .

push-parent:       ## Push the image
	docker login
	docker push ${PARENT_IMAGE_NAME}

compose-build:
	docker-compose build

##up:                 Run the vnc containers
up: compose-build
	docker-compose up

##down:               Kill and delete the containers
down:
	docker-compose down
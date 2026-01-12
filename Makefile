all: docker

check: docker
	darjeeling repair repair.yml

clean:
	rm -f darjeeling.log.*
	rm -f bugzood.log
	rm -rf patches

docker:
	docker build -t ros_repair:vulnerable docker

.PHONY: check clean docker


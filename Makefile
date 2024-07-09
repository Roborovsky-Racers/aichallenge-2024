# build base image
up:
	docker compose up
gpu-up:
	docker compose -f docker-compose.yml -f nvidia.yml up
down:
	docker compose down
build:
	docker compose build

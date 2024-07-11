# build base image
up:
	docker compose up -d
	@echo ">> entering container with tmux..."
	@docker exec -it aic /usr/bin/tmux
	docker compose down
gpu-up:
	docker compose -f docker-compose.yml -f nvidia.yml up -d
	@echo ">> entering container with tmux..."
	@docker exec -it aic /usr/bin/tmux
	docker compose down
down:
	docker compose down
build:
	docker compose build
build-no-cache:
	docker compose build --no-cache
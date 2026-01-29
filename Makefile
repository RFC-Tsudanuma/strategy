venv:
	./init_env.sh

format:
	./run_ruff.sh

test:
	uv run pytest
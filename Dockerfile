FROM python:3.12-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY pyproject.toml README.md ./
COPY backend ./backend
COPY configuration ./configuration
COPY configurations ./configurations
RUN pip install --no-cache-dir .
COPY backend/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data
ENV DROVER_CONFIG_DIR=/data DROVER_DB_PATH=/data/drover.sqlite3
EXPOSE 8000
CMD ["/entrypoint.sh"]

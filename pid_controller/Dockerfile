FROM docker.io/python:3.12-slim-bookworm as DEV

WORKDIR /app
COPY . /app

RUN pip install --no-cache-dir -r requirements.txt

FROM DEV as PROD
ENTRYPOINT ["python3", "-u", "main.py"]
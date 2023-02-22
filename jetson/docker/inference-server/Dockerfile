FROM nvcr.io/nvidia/l4t-tensorflow:r32.7.1-tf2.7-py3

WORKDIR /app

COPY app .
RUN pip3 install -r requirements.txt

COPY app .

CMD gunicorn -b 0.0.0.0:80 main:app

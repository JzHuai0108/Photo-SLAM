FROM photoslam:latest

# We use pi for user name here, but you can pick any name you like.
ARG UID
RUN adduser --uid $UID --disabled-password --gecos "" pi
USER pi

WORKDIR /home/pi

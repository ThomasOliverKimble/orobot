FROM benjamindeleze/webots-test:R2022b15LGA
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH
# RUN cd $PROJECT_PATH/controllers/OrobotController && make clean
# RUN cd $PROJECT_PATH/controllers/OrobotController && make

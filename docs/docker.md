# Using Docker to run RDS

Follow this guide to run RDS on docker.

## Requirements

The only dependency that you should install is the standard Docker Engine:

- Install Docker Engine: https://docs.docker.com/engine/install/

## Build local docker image
```
docker build -t rds:latest .
```

## Run

Once the docker image is built, you can run the software using the following:

```
docker run -i -w <DIRECTORY> rds:latest <COMMAND>
```

where:
- `<DIRECTORY>` is the base directory to execute the command
- `<COMMAND>` is the executable string

For example, to run the simulation, located on `/rds`:

```
docker run -i -w /rds rds:latest ./build/baseline_crowd_rds_5 7
```


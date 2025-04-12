# SucréLA (READ ONLY, MOVED TO GITLAB)

This is an archived, out-of-date and read-only copy of the SucréLA project.

The project is now hosted on **GitLab** => [https://gitlab.com/yannsionneau/SucreLA](https://gitlab.com/yannsionneau/SucreLA)

## Building

The project Python's dependencies are handled using Pipenv.
However, you must install [nextpnr-ecp5](https://github.com/YosysHQ/nextpnr) and [Yosys](https://github.com/YosysHQ/yosys) if you wish to build the
projet from source.

```
pipenv install --ignore-pipfile
pipenv run python3 ./sucrela.py --build
```

## Flashing the OrangeCrab

```
pipenv run python3 ./sucrela.py --load
```

## Runing simulation

```
pipenv run python3 ./sucrela.py --sim
```

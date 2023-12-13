# SucreLA

## Building

The project dependencies are handled using Pipenv.

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

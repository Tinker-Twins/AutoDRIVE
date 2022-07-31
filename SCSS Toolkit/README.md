# Web Application

This directory hosts the Smart City Manager (SCM) Webapp for AutoDRIVE, which can be used to develop smart-city solutions for intelligent transportation systems.

## Install

Flask

```bash
$ pip install flask
```

SQLite3

```bash
$ pip install sqlite3
```

## Setup

Unix Bash (Linux, Mac, etc.):

```bash
$ export FLASK_APP=scm_app
$ export FLASK_ENV=development
```

Windows CMD:

```bash
> set FLASK_APP=scm_app
> set FLASK_ENV=development
```

## Run

1. Initialize SCM Database:

    If the `database.db` is not already initialized, run the `init_db.py` script to do so:

    ```bash
    python3 init_db.py
    ```

2. Launch SCM Webapp:

    Launch back-end of the SCM Webapp by running the flask app:
    
    ```bash
    cd <path/to/autodrive_scm/>
    flask run
    ```

    Launch front-end of the SCM Webapp by redirecting to the link populated in the terminal using a browser application.

3. Launch SCM Server:

    Launch the SCM Server to communicate with the AutoDRIVE Simulator.
    
    ```bash
    python3 server.py
    ```

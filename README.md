# Automated E-CoDrive

Subset di E-CoDrive dedicato alla simulazione automatizzata headless con CARLA `0.9.13`, SUMO e Autoware Mini.

## Contenuto

- `ecodrive/simulation/automated_simulation.py`: API principale, `simulate(...)`.
- `ecodrive/simulation/automated_test.py`: esempio di lancio batch.
- `ecodrive/scenario/sumo_route_tools.py`: generazione scenari, bootstrap CARLA/SUMO e gestione Autoware.
- `ecodrive/cosimulation/`: runner SUMO-CARLA usati dal workflow automatico.
- `ecodrive/analysis/battery_plots.py`: parsing output energetici e plot.
- `bootstrap_templates/` e `scripts/setup_carla.sh`: patch/setup della folder CARLA `0.9.13`.
- `autoware_mini_docker_compose/`: container Autoware Mini.
- `carla/CARLA_0.9.13/`: installazione CARLA usata dall'automated simulation.

La dashboard Streamlit originale e il workflow CARLA `0.9.15` non sono inclusi.

## Setup

Da questa cartella:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
./scripts/setup_carla.sh
```

Se l'interprete usato per lanciare il progetto non riesce a importare la Python API di CARLA `0.9.13`, configura un interprete compatibile:

```bash
export CARLA_PYTHON_0_9_13=/path/to/python
```

L'interprete CARLA deve poter importare almeno `carla`, `flask`, `lxml`, `traci`, `sumolib` e `setuptools`.
L'interprete principale del progetto può essere diverso: in quel caso configura
`CARLA_PYTHON_0_9_13` nella shell o nelle variabili d'ambiente della Run Configuration.
Per esempio, con gli ambienti Conda locali:

```bash
export CARLA_PYTHON_0_9_13=/home/manfr/anaconda3/envs/ecodrive/bin/python3.8
python -m ecodrive.simulation.automated_test
```

Il runner CARLA `0.9.13` incluso usa un'estensione binaria Python 3.7. Un interprete
più recente può terminare con un segmentation fault anche quando tutte le dipendenze
elencate sono installate.

## Uso

Esempio:

```bash
python -m ecodrive.simulation.automated_test
```

Oppure importa direttamente:

```python
from ecodrive.simulation.automated_simulation import simulate
```

La simulazione automatizzata usa solo `CARLA_0.9.13`.

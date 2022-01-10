# Follow-target simulation for PX4

## Installing dependencies
For the python script:

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
```

For PX4:
Install the firmware compilation toolchain together with a simulator follow the [PX4 instructions](http://docs.px4.io/master/en/dev_setup/dev_env.html)



## Running
In first terminal:

```bash
make px4_sitl jmavsim
```

In second terminal:

```bash
python3 follow-target-sim.py
```

## Formatting for contributions with 

```
autopep8 --in-place --aggressive --aggressive -r .
```
## Welcome to the UIUC Hand Python SDK

#### Install On Ubuntu
- `python -m venv test_env`
- `source test_env/bin/activate`
- `pip install dynamixel_sdk numpy`
- `sudo chmod 777 /dev/serial/by-id/*`
- `python main.py`

#### Install On Windows
- Use Windows powershell
- We recommend to create a virtual environment and pip install the code:
- `Set-ExecutionPolicy Unrestricted -Scope Process`
- `python -m venv test_env`
- `.\test_env\Scripts\activate.ps1`
- `pip install dynamixel_sdk numpy`
- `python main.py`

Please see main.py for further details.  It should be easy to read.  :)

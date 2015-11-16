This is a python api to control spiri, using the services and actions provided by the ros library.
To install this on your system, run:
```
python setup.py install
```
from the root of this repository.
To use this in a python script, include the following line:
```
from spiripy import api
spiri = api.SpiriGo()
```
`spiri` will then be a SpiriGo instance with the following methods:
+ `armAndTakeoff(height)`

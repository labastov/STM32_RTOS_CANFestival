## Форк <https://github.com/fbsder/canfestival> т. к. похоже это самый живой форк оригинала

Подправлен генератор словаря *ObjdicEdit*, теперь он работает с текущими библиотеками Python.
Включена возможность указать `can ID` устройства при генерации словаря.


`ObjdicEdit.py` протестирован с:

````
Python 2. 7. 13 (v2. 7. 13:a06454b1afa1, Dec 17 2016, 20:53:40) [MSC v. 1500 64 bit (AMD64)] on win32
Type "help", "copyright", "credits" or "license" for more information.

>>> import wx
>>> wx. __version__

'4. 0. 7. post2'

>>> quit()
````

также `ObjdicEdit. py` должен работать в debian10.

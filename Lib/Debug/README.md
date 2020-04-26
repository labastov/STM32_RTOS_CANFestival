Поддерживается вывод отладочой информации через SWD и UART для чего введены две функции:

``` c
DEBUG_NAME_FUNCTION; // макрос вывода имени функции
debug_printf("TempSensorCount:%d\n", ss); // вывод отладочной информации

```

Все сделан на базе _HAL_.

## необходимые требования

1. должен быть установлен STM32CubeMX.
2. расширение cortex-debug для VSC
3. gcc

## Инструкция

1.Создайте проект STM32CubeMX активируйте режи SWD/Jtag (раздел SYS/DEBUG) или  соответствующий порт UART.

В закладке `Project Manager` укажите генерировать `Makefile` , раздельно генерировать *.с и *.h файлы.

2. Скопируйте папки `.vscode` , `Documents` , `Support` , `Lib` в новый проект.

3.В `main.h` вставьте следующие определения:

``` C
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "usart.h"  // при использовании UART
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* объявляем режим отладки через SWO, только в этом случае
   функции включаются в пошивку */
// #define DEBUG_SWO

/* если используете отладку через UART. не забываем правильно указывать  номер
   используемого порта */
#define DEBUG_UART huart1

/* USER CODE END Private defines */
```

Определения _DEBUG_SWO_ и  _DEBUG_UART_PPORT_ должны использоваться по переменно (иначе приоритет за SWO) или не одно, тогда вывод отладочной информации отключается в коде

4. Откорректируйте `Makefile` сгенерированный STM32CubeMX

* укажите явно *.с* файлы добавленные в проект, так же надо будет явно указать се файлы которые вы добавите (у меня не получилось сборка по шаблону)

``` c
# C sources

C_SOURCES =  \
Lib/debug.c \
Lib/printf-stdarg.c \

```

* Добавьте директории где лежат дополнительные хедеры

``` c
# C includes
C_INCLUDES =  \
-ILib \
-IInc \

```

* Добавьте информацию о размерах прошивки вконец  раздела ключ `-Wl,--gc-sections` 

``` c
# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) \
-specs=nano.specs \
-T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections \
-Wl,--print-memory-usage
```

* включите вывод данной информации в консоль.строки `@echo $<` в блоке кода

``` c
#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo $<
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo $<
	$(AS) -c $(CFLAGS) $< -o $@

```

5. откорректируйте пути к внешним программам в `settings.json`

## Проблемы

По какой-то причине не корректно работает `HAL_Delay` - проверь установку фьюза wdg_sw

## Примечания

1. Заметил, что нужно порт UART_TX подтягивать резистором к питанию иначе до момента инициализации
порта через него будет сыпаться мусор, т.к до инициализации порт висит в воздухе.

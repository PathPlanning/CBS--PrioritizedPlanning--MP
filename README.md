# [English documentation](README-EN.md)
# Push-and-Rotate--CBS--PrioritizedPlanning

В данном проекте приводятся реализации различных алгоритмов решающих задачи планирования траекторий для группы агентов. Рассматриваются алгоритмы [Conflict based search](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239), [Enhanced conflict based search](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875), [Push and rotate](https://pdfs.semanticscholar.org/0a84/5fa6530f84b5df50d652a5e4eecc38d77681.pdf) и [Prioritized planning](https://arxiv.org/pdf/1409.2399.pdf), а также некоторые их модификации.

## Сборка и запуск
Для сборки проекта можно использовать cmake с помощью файла CMakeLists.txt, размещенного в репозитории. Также файлы проекта могут быть открыты и скомпилированы в Qt Creator с конфигурациями, заданными в файле PathPlanning.pro.

Данные поступают в виде файлов в формате XML: одного основного файла и одного или нескольких дополнительных. В качестве первого аргумента командной строки передается название основного входного файла, в котором приводится описание среды, алгоритма и даются ссылки на дополнительные входные файлы с агентами. Результатом работы является выходной файл в формате XML. Примеры оформления входных и выходного файлов можно найти в разделе Examples.

## Формат входных данных
Основной файл содержит разделы map и options:

### Раздел map - задание карты. Содержит следующие тэги:
- width, height - ширина и высота карты (в клетках) - целые числа
- grid - сама карта, 0 означает свободную клетку, а 1 - препятствие. Ряды обособляются тэгом row, количество рядов должно быть равно height, а количество символов в каждом ряду должно быть равно width.

### Раздел options - выбор параметров алгоритма и тестов. Содержит следующие тэги:
- algorithm - используемый алгоритм. Может принимать следующие значения:
    1. cbs - Conflict based search
    2. ecbs - Enhanced conflict based search. В алгоритме на верхнем уровне используется вторичная эвристика h3 из [статьи](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875), а на нижнем вторичная эвристика, равная количеству вершинных конфликтов на уже построенном участке траектории до текущей вершины
    2. push_and_rotate - Push and rotate
    3. prioritized_planning - Prioritized planning
- low_level - алгоритм, используемый в поиске нижнего уровня в алгоритмах CBS и Push and rotate. Может принимать следующие значения:
    1. astar - алгоритм [A*](https://www.cs.auckland.ac.nz/courses/compsci709s2c/resources/Mike.d/astarNilsson.pdf)
    2. sipp - алгоритм [SIPP](https://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/viewFile/8911/8875) (дискретная версия)
- agents_file - общий префикс названий входных файлов с описанием агентов
- tasks_count - количество входных файлов с описанием агентов: рассматриваются файлы с названиями вида agents_file-n.xml, где n принимает значения от 1 до tasks_count
- agents_range - в атрибутах min и max данного тега указывается минимальное и максимальное количество агентов для тестирования. При single_execution=`false`, количество агентов увеличивается от min до max, и алгоритм запускается на соотвествующем подмножестве агентов. Тестирование прекращается, если алгоритм работает дольше заданного лимита по времени или не находит решения.
- maxtime - максимальное время работы алгоритма в миллисекундах
- with_perfect_h - будет ли производиться предпосчет кратчайших путей от всех вершин до конечных позиций агентов для вычисления оптимальной эвристики в A\* (`true` или `false`, учитывается для алгоритмов CBS и Prioritized planning).
- with_cat - будет ли использоваться Conflict avodance table (`true` или `false`, учитывается если выбран алгоритм CBS)
- with_card_conf - будут ли учитываться кардинальные конфликты (описываются [здесь](https://pdfs.semanticscholar.org/c072/38579a95c424707dbe855efba189cce68650.pdf)). Принимает значения `true` или `false`, учитывается если выбран алгоритм CBS.
- with_bypassing - будет ли производиться обход конфликтов (conflict bypassing). Описывается [здесь](http://faculty.cse.tamu.edu/guni/Papers/ICAPS15-Eli.pdf), принимает значения `true` или `false`, учитывается если выбран алгоритм CBS.
- with_matching_h - будет ли вычисляться эвристика на вершинах дерева обхода CBS, основанная на максимальном паросочетаннии в графе кардинальных конфликтов. Описывается [здесь](http://idm-lab.org/bib/abstracts/papers/icaps18a.pdf) как ICBS-h1, принимает значения `true` или `false`, учитывается если выбран алгоритм CBS. При использовании этой опции, опция with_card_conf фиксируется равной `true`.
- with_disjoint_splitting - будет ли производиться disjoint splitting. Описывается [здесь](http://idm-lab.org/bib/abstracts/papers/icaps19a.pdf), принимает значения `true` или `false`, учитывается если выбран алгоритм CBS. При использовании этой опции, опция with_card_conf фиксируется равной `true`.
- pp_order - задает правило, согласно которому выбираются приоритеты агентов в prioritized_planning. Может принимать следующие значения:
    - 0 - агенты обрабатываются в том же порядке, в котором они указаны в файле
    - 1 - агенты обрабатываются в порядке увеличения расстояния от стартовой позиции до конечной
    - 2 - агенты обрабатываются в порядке уменьшения расстояния от стартовой позиции до конечной
- parallelize_paths_1 - требуется ли применять процедуру параллеллезации путей, построенных Push and rotate (без этой опции в каждый момент времени двигается только один агент). Принимает значения `true` или `false`, учитывается только для алгоритма Push and rotate.
- parallelize_paths_2 - требуется ли дополнительно параллелизовать пути, построенные Push and rotate (увеличивает коэффициент сжатия, но замедляет работу алгоритма). Принимает значения `true` или `false`, учитывается только для алгоритма Push and rotate. При использовании этой опции, опция parallelize_paths_1 фиксируется равной `true`.
- single_execution - `true` или `false`, если выбрано значение `true`, алгоритм будет запущен только один раз для первого файла с агентами, с количеством агентов равным значению атрибута max в agents_range. При этом будет отличаться формат выходного файла (см. раздел "Формат выходных данных").
- logpath - каталог, в который будет сохранен отчет (не обязательный параметр, по умолчанию, отчет сохраняется в тот же каталог, в котором находится входной файл)
- logfilename - название файла с отчетом (не обязательный параметр, по умолчанию, название файла с отчетом получается из названия входного файла дописыванием строки "_log")

### Файлы с агентами
Каждому агенту соответствует собственный тег agent со следующими атрибутами:
- id - идентификатор агента
- start_i, start_j - координаты стартовой позиции агента (клетки нумеруются с 0, клетка (0, 0) это верхний левый угол карты, первая координата соответствует номеру строки, а вторая номеру столбца)
- goal_i, goal_j - координаты конечной позиции агента

## Формат выходных данных
Выходной файл так же содержит разделы map и options, содержание которых совпадает со входным файлом, а также раздел log, в котором содержится сам отчет. Он содержит
тэг mapfilename с названием основного входного файла, и ряд других тегов в зависимости от значения параметра single_execution.

В случае, если single_execution=`false`:

- aggregated_results - результаты тестирования. Для каждого количества агентов указывается тег result со следующими атрибутами:
    1. agents_count - количество агентов
    2. success_count - количество тестов (среди tasks_count тестовых файлов) для которых алгоритм построил корректное решение в заданное время
    3. makespan - количество итераций до того как последний из агентов закончит движение (усредненное)
    4. flowtime - суммарное количество действий, совершенных агентами с учетом остановок (усредненное)
    5. time - время работы алгоритма (усредненное)

В случае, если single_execution=`true`:

- taskfilename - название дополнительного входного файла с агентами
- summary - информация о построенном решении. Содержит атрибуты agents_count, makespan и flowtime, которые определяются так же, как указано выше
- для каждого агента указывается отдельный тег agent со следующими атрибутами:
    - id - идентификатор агента
    - start.x, start.y - координаты стартовой позиции агента
    - goal.x, goal.y - координаты конечной позиции агента

    Также в этот тег вкладывается тег path с атрибутом pathfound, принимающим значения `true` или `false` в зависимости от того, было ли найдено решение, в который в свою очередь вкладывается несколько тегов section. Каждый такой тег соответствует одному действию агента и содержит следующие теги:
    - id - идентификатор секции
    - start.x, start.y - позиция агента перед совершением действия
    - goal.x, goal.y - позиция агента после совершения действия (эта позиция либо совпадает со стартовой либо является соседней с ней)
    - duration - продолжительность действия (всегда равна 1)

Тестовое задание:

Требуется реализовать логику работы самолётов, а именно - простейшую физику полёта и искусственный интеллект самолёта,
подчиняющийся игроку.
В игре уже присутствует простейшая логика авианосца и вся необходимая графика.

Основные требования:
- на борту корабля должно быть 5 самолётов
                - будучи на борту самолёты никак не отрисовываются в мире
- по правому клику мыши в воздух поднимается очередной самолёт
                - самолёт должен начать отрисовываться в мире
                - самолёт должен начать взлетать вдоль палубы корабля, медленно набирая скорость
- самолёт при движении должен учитывать следующие параметры
                - время полёта (настроечный параметр)
                - максимальная скорость (настроечный параметр)
                - максимальная угловая скорость (настроечный параметр)
- по истечении времени полёта самолёт должен вернуться на корабль
                - самолёт должен долететь до своего корабля, только после этого он считается севшим
                - после посадки самолёт больше не рисуется в мире
                - после посадки самолёт может быть снова поднят в воздух только по истечении определённого времени, требующемся для заправки самолёта (настроечный параметр)
- по левому клику мыши назначается новая цель для самолётов
                - самолёты, находящиеся в воздухе, летят в направлении цели и начинают кружить вокруг неё

Управление в игре:
- WASD и стрелки управляют движением корабля
- правый клик мыши поднимает очередной самолёт в воздух
- левый клик мыши назначает цель для самолётов
- пробел перезапускает игру

Что будет оцениваться:
- физичность полёта самолётов с точки зрения обычного игрока
- выполнение требований к поведению самолётов
- читаемость и аккуратность кода

Пояснения по структуре проекта:
- код-заготовка этой игры находится в файле game.py. Здесь должна происходить основная работа.
                - на данный момент настроечные параметры находятся в классе Params
                - логика работы корабля находится в классе Ship. Сейчас она реализована только на базовом уровне, необходимом для тестирования игры, но не для самой игры, и сильно страдает от недостатка физичности. Доработка поведения корабля не входит в тестовое задание.
- "движок" находится в библиотека framework.pyd
- при необходимости в проект можно добавлять новые файлы

Краткая справка по доступным для кода игры функциям "фреймворка":
- framework.createShipModel() - создаёт новую модель корабля и возвращает ее
- framework.createAircraftModel() - создаёт новую модель самолёта и возвращает ее
- framework.destroyModel( model ) - удаляет модель, созданную любой из двух предыдущих функций
- framework.placeModel( model, posX, posY, yaw ) - располагает модель в мире в указанных координатах и повёрнутой под нужным углом

Перед выполнением задания рекомендуется ознакомиться с материалами - https://gamedevelopment.tutsplus.com/understanding-steering-behaviors-pursuit-and-evade--gamedev-2946t

Требования к технической реализации:
- Реализация на Python 2.7
- Использование сторонних библиотек, кроме framework.pyd не допускается
- Изменения в framework.pyd также не допускаются


Критерии оценки:
- Полнота и качество реализации
- Физичность полёта самолётов с точки зрения обычного игрока
- Оформление кода

# Поиск рук по цвету кожи (без нейросетей и файлов моделей).
# Плохо находит — подстрой SKIN_THRESHOLD под своё освещение.

# Порог в LAB: (L_min, L_max, A_min, A_max, B_min, B_max)
SKIN_THRESHOLD = (20, 75, 10, 40, 10, 45)

MIN_AREA_RATIO = 0.01  # области меньше 1% кадра не считаем
MAX_AREA_RATIO = 0.60  # и больше 60% кадра тоже
MAX_HANDS = 2


def _val(x):
    """В старых прошивках OpenMV у области это методы (b.pixels()), в новых — готовые
    значения (b.pixels). Вызов значения как функции ронял камеру: 'int' object isn't
    callable — причём только когда в кадре появлялось что-то цвета кожи."""
    return x() if callable(x) else x


def detect(img):
    area = _val(img.width) * _val(img.height)
    blobs = img.find_blobs([SKIN_THRESHOLD],
                           pixels_threshold=int(area * MIN_AREA_RATIO),
                           merge=True)
    boxes = []
    for b in blobs:
        if _val(b.pixels) > area * MAX_AREA_RATIO:
            continue
        boxes.append(tuple(_val(b.rect)))
        if len(boxes) >= MAX_HANDS:
            break
    return boxes


def draw(img, boxes):
    for r in boxes:
        img.draw_rectangle(r, color=(0, 255, 0))

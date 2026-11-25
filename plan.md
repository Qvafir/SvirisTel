# План реализации SvirisTel (svt)

## Разделение уровней обработки данных

1. **Фреймер COBS** - кодирует кэш Header, Trailer и SDU при uplink, либо декодирует принимаемые сегменты при downlink.
2. **Сереализатор PDU** - разбирает PID, SDL, SDU, CRC из принятого потока декодированных байт или складывает незакодированные PID, SDL, CRC в кэш Header, Trailer.
3. **Валидатор** - вычисляет и сравнивает CRC, сравнивает SDL с фактическим размером принятого кадра, обрабатывает исключения.

## Машина потокового кодирования

Имется 3 сегмента исходных данных (HEAD, SDU, TRAIL), необходимо произвести потоковое кодирование по алгоритму COBS с кодом разделения кадра (нулем) в начале и конце кадра с применением микробатчинга в промежуточном буфере произвольного размера.

Непрерывный доступ к сегментам данных реализуется с применением виртуальной конкатенации, преобразуя его в виртуальный PDU

соглашения:
- Кадр имеет вид COBS(PDU) | 00.
- PDU[0] — заглушка (0x00) под первый OHB. Контракт: pdu_size ≥ 1 и pdu_data(0) == 0.
- Позиция pdu_size — виртуальный ноль, терминатор последнего блока. Он никогда не копируется в буфер.
- Окно — это диапазон [pdu_window_start, pdu_window_end). Его размер равен min(свободное место в сегменте, остаток PDU). Окно не может выйти за pdu_size.
- segment_end — число записанных байт в буфере.

данные:

``` c
pdu_data(pos)            // виртуальная конкатенация HEAD|SDU|TRAIL
frame_segment[segment_size]

pdu_size
pdu_window_start         // первый ещё не скопированный байт PDU
pdu_window_end           // конец окна (не включая)
segment_size             // константа, >= 2 (лучше >= 3)
segment_end              // число записанных байт буфера

ohb_pos                  // позиция PDU, где должен стоять очередной OHB
insert_mode              // false: в ohb_pos лежит ноль (placeholder или данные 0x00), OHB его заменяет
                         // true: в ohb_pos данные, OHB нужно вставить перед ними (после кода 0xFF)
```

примитивы:

``` c
// окно = свободное место в буфере, но не дальше конца PDU
update_window_end()
  pdu_window_end = min(pdu_window_start + (segment_size - segment_end), pdu_size)

// расстояние от позиции кода base до ближайшего нуля (или конца PDU),
// просмотр base+1 .. base+254; если не найдено, то 0xFF
// результат всегда лежит в 1..0xFF
find_in_pdu(base):
  for i = base+1 .. base+254:
    if i == pdu_size or pdu_data(i) == 0:
      return i - base
  return 0xFF
// реализация: memchr по кускам HEAD/SDU/TRAIL с лимитом 254 байта

is_end_of_pdu()
  return pdu_window_start >= pdu_size

// копировать всё окно в буфер
copy_window()
  segment_end      += pdu_window_end - pdu_window_start
  pdu_window_start  = pdu_window_end

// вернуть старт окна назад (pdu_pos < pdu_window_start, байт уже в буфере)
reset_window_start_to(pdu_pos)
  segment_end      = map_pdu_pos_to_segment(pdu_pos)
  pdu_window_start = pdu_pos
  // pdu_window_end пересчитает следующий write_byte_to_segment

// допустимо для pdu_pos в текущем сегменте, после последней вставки
map_pdu_pos_to_segment(pdu_pos)
  return segment_end + (pdu_pos - pdu_window_start)     // знаковая арифметика

write_byte_to_segment(byte)
  frame_segment[segment_end] = byte
  segment_end += 1
  update_window_end()

set_byte_to_segment(pos, byte)       // segment_end не меняется
  frame_segment[pos] = byte

flush_segment()
  send(frame_segment, segment_end)   // см. замечание о владении буфером
  segment_end = 0
  update_window_end()
```

кодирование сегмента:

```c
encode_segment()
  copy_window()                                  // сначала всё окно копируется как есть

  while (ohb_pos < pdu_window_end)               // байт ohb_pos уже в буфере сегмента
                                                 // (при ohb_pos == pdu_size цикл тоже заканчивается)
    if (!insert_mode)
      base = ohb_pos                             // в ohb_pos лежит ноль
    else
      base = ohb_pos - 1                         // «виртуальный» код стоит перед ohb_pos

    ohb_val = find_in_pdu(base)                  // 1..0xFF

    if (!insert_mode)
      // заменяем ноль на код
      set_byte_to_segment(map_pdu_pos_to_segment(ohb_pos), ohb_val)
    else
      // врезаем код между двумя кусками PDU
      reset_window_start_to(ohb_pos)
      write_byte_to_segment(ohb_val)             // место есть: байт ohb_pos был в буфере
      copy_window()                              // докопировать то, что теперь помещается

    ohb_pos    = base + ohb_val                  // позиция следующего кода
    insert_mode = (ohb_val == 0xFF)              // после 0xFF следующего нуля нет, код вставляется
```

Если после кода 0xFF ohb_pos == pdu_size, цикл закончится сам. Лишний код не вставляется.
Если ohb_pos == pdu_window_end и буфер полон, вставка откладывается. Код встанет первым байтом следующего сегмента.

основная программа:

``` c
init(pdu, segment_size):
  state = START

step():                            // один вызов = один отправленный сегмент
  switch (state):

  IDLE:
    noop()

  START:
    segment_end      = 0
    pdu_window_start = 0
    ohb_pos          = 0
    insert_mode      = false
    update_window_end()
    state = CONTINUE
    goto CONTINUE_BODY             // PDU может поместиться в первый же сегмент

  CONTINUE:
  CONTINUE_BODY:
    encode_segment()

    if (is_end_of_pdu() and ohb_pos >= pdu_size)   // PDU скопирован и все OHB расставлены
      if (segment_end < segment_size)
        write_byte_to_segment(0x00)                // конечный разделитель влез
        flush_segment()
        state = IDLE
      else
        flush_segment()                            // буфер полон, разделитель уйдёт следующим
        state = END
    else
      flush_segment()

  END:
    write_byte_to_segment(0x00)
    flush_segment()
    state = IDLE
```

## Машина потокового декодирования

Задача:

Реализовать обработку кадров COBS в непрерывном потоке байт с применением микробатчинга в виде синхронной обработки отдельных аккумулируемых сегментов потока байт (вместо побайтовой обработки).

Используемые термины:
Поток - непрерывная последовательность данных.
Сегмент - последовательная часть потока произвольного размера.
Кадр - непрерывный набор данных закодированный протоколом COBS (начинается и заканчивается нулевым байтом).

Данные:

``` c
seg_size    // размер текущего сегмента
seg_head    // позиция первого ещё не переданного/не отброшенного байта

ohb_pos     // позиция следующего OHB относительно начала текущего сегмента
ohb_forced  // следующий OHB форсированный (вставлен в разрез PDU, не заменяет 0)
pdu_len     // сколько байт PDU уже передано парсеру в текущем кадре

in_frame    // 1 — идёт приём кадра, 0 — ожидание разделителя (синхронизация)
```

Примитивы:

``` c
// передать парсеру size байт от seg_head; возвращает ошибку при отказе (back-pressure, переполнение)
handle(size):
  seg_head += size

// отбросить size байт от seg_head
skip(size):
  seg_head += size

// первый 0x00 в полуоткрытом диапазоне [seg_head, end); если нет — вернуть end
find_null(end)

// доступ к байту сегмента (запись допустима)
byte(pos)

// завершить кадр: in_frame = 0, уведомить парсер (COMPLETE: принять PDU, ERROR: откатить)
reset(status)

// передать парсеру кусок PDU с контролем лимита; при отказе — переход в fail
emit(size):
  if (size == 0) return
  if (pdu_len + size > MAX_PDU or handle(size) == ERR) goto fail
  pdu_len += size

// начать новый кадр с seg_head (сразу за разделителем)
begin_frame():
  ohb_pos    = seg_head
  ohb_forced = 1          // первый OHB гарантированно форсированный
  pdu_len    = 0
  in_frame   = 1
```

Основная программа:

``` c
init():
  in_frame = 0

work(segment, seg_size):               // всегда потребляет сегмент целиком, возвращает seg_size
  seg_head = 0

  while (1)
    if (!in_frame)
      // синхронизация: отбросить всё до разделителя включительно
      null_pos = find_null(seg_size)
      skip(null_pos - seg_head)
      if (null_pos >= seg_size)
        return(seg_size)
      skip(1)                          // сам разделитель
      begin_frame()

    // ЕДИНСТВЕННЫЙ поиск нуля на сегмент: ближайший 0 — это либо разделитель, 
    // либо нелегитимный 0 (оба случая — граница кадра)
    null_pos = find_null(seg_size)

    // идем по цепочке OHB; в [seg_head, null_pos) нулей нет, поэтому byte(ohb_pos) != 0
    while (ohb_pos < null_pos)
      code = byte(ohb_pos)             // читаем ДО подмены
      if (ohb_forced)
        emit(ohb_pos - seg_head)       // PDU до OHB (может быть пустым)
        skip(1)                        // пропустить OHB
      else
        byte(ohb_pos) = 0              // OHB замещает нулевой байт данных
      ohb_forced = (code == 0xFF)
      ohb_pos   += code

    if (null_pos >= seg_size)
      // нулей нет — кадр продолжается в следующем сегменте
      emit(seg_size - seg_head)
      ohb_pos -= seg_size              // здесь ohb_pos >= seg_size гарантировано
      return(seg_size)

    if (ohb_pos != null_pos)           // ohb_pos > null_pos: 0 попал внутрь блока данных
      goto fail

    // ohb_pos == null_pos: валидный конец кадра
    emit(null_pos - seg_head)
    skip(1)                            // разделитель
    if (pdu_len > 0)
      reset(COMPLETE)
    else
      in_frame = 0                     // пустой кадр (00 00): простой линии, не событие
    begin_frame()                      // следующий кадр начинается сразу за разделителем
    continue

  fail:
    reset(ERROR)                       // парсер откатывает частично принятый PDU
    if (null_pos >= seg_size)
      skip(seg_size - seg_head)
      return(seg_size)                 // in_frame = 0: ждем разделитель в следующих сегментах
    skip(null_pos + 1 - seg_head)      // 0 — граница, следующий кадр начинается за ним
    begin_frame()
```

## Сереализатор PDU

Сереализация осуществляется с применением виртуальной конкатенации.

Виртуальная конкатенация - реализует непрерывный доступ к сегментам данных находящимся в разных участках памяти (массивах).
Доступ к данным выглядит так, как после объединения нескольких массивов данных в единый буфер, но при этом без копирования.

Всю логику возможно реализовать в линейном коде с несколькими проверками if/else (branchless-friendly) и использованием сверхбыстрых аппаратно-оптимизированных функций стандартной библиотеки (memchr и memcpy).

TODO: написать псевдокод 

## Десериализатор PDU

Десериализатор PDU оптимальнее всего реализовать с применением машины состояний.

TODO: написать псевдокод 

## Оркестратор

Комбинирует программные модули:
- Машина потокового кодирования COBS
- Машина декодирования COBS
- Сереализатор PDU
- Десереализатор PDU

Управляет их жизненным циклом.
Интегрирует их совместную работу.
Обрабатывает промежуточные исключения.

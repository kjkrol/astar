---
name: hpastar
description: Model solvera hpastar (github.com/kjkrol/hpastar) — korzeń Solver/Search nad kontraktem graph.Implicit, z graph.Layout HPA*; silniki w internal (astar, dijkstra, hierarchy, heap, dict); niezmienniki, pułapki, odrzucone pomysły, pomiar. Czytaj przy zmianie solvera, kontraktu grafu, silników lub hierarchii, i z grama (plugins/navigation), gdy chodzi o to, co planner dostaje od solvera.
---

# hpastar

A* po dowolnym grafie stanów (`graph.Implicit`); z `graph.Layout` długie ścieżki liczy HPA*
(Botea, Müller, Schaeffer 2004). API i kontrakt opisuje godoc korzenia i pakietu `graph`, liczby
`BENCHMARKS.md`. Do v1.3.0 moduł nazywał się `github.com/kjkrol/astar`. Jedyny użytkownik to
gram, `plugins/navigation/internal/pathfind`.

## Model

- `Search.Path` wspina się po hierarchii, gdy jest `Layout`, hierarchia gotowa, a końce dalekie:
  `Climb.Route`, potem `Refine` leniwie do `dst`. Bliskie końce, stan poza płaszczyzną i
  niegotowa hierarchia idą dokładnym A*. Fałsz tylko wtedy, gdy ścieżki nie ma.
- Parę bez ścieżki wspinaczka odrzuca z etykiet bez szukania. Dokładny A* po `Patience()`
  rozwinięciach pyta etykiet i kończy bez zalewu.
- Klastry poziomu 1 to bloki `side²` współrzędnych, wyżej bloki 4×4, a szczyt ma ≤ 16 klastrów.
  Budowa idzie zadaniami w krokach, jedno naraz: nieaktualne klastry poziomu 1, potem etykiety,
  potem najniższy wyższy poziom.

## Układ

- Pakiet na funkcję, bez aliasów i cykli: `graph` ← `heap`, `dict` ← `astar`, `dijkstra` ←
  `hierarchy` ← korzeń. Korzeń niczego nie reeksportuje. Pakiet wewnętrzny eksportuje tylko to,
  czego używa inny pakiet albo jego testy.
- `astar` i `dijkstra` się nie znają, każdy ma własny `node[T]`. `hierarchy` woła je przez metody.
- `internal/testmap` importuje tylko `graph` i `dict`: import silnika zamknąłby cykl z jego
  testami. `examples/maps` ma własną kopię generatora, bo stoi na samym publicznym API, więc
  zmiana generatora idzie do obu.
- Zmienna lokalna nazwana jak pakiet (`graph`, `dict`) przesłania go w swoim zasięgu.

## Niezmienniki

- **Remisy jak w `container/heap`.** Kopiec binarny z sitem niosącym dziurę porównuje tak samo
  jak `container/heap`, więc ścieżki są te same od v1.1.3. Pilnuje tego `testdata/paths.golden`.
  Zmiana remisów przepisuje go i zmienia `routes.golden` grama, bo tłum przeplanowuje inaczej.
- **Węzły w tablicach po `int32`**, bez wskaźników, żeby GC ich nie skanował. `alloc` może
  przenieść tablicę, więc pola bieżącego węzła kopiuje się przed pętlą następników, nigdy
  `&nodes[i]`.
- `existing.G <= G` nie jest poprawą: pierwszy rodzic zostaje. Stan zamknięty osiągnięty taniej
  otwiera się jako świeży węzeł.
- **Znaczniki słownika** (`dict`): każde szukanie przesuwa znacznik 1..255 i czyści 1/255
  slotów, więc nic nie wymaga pełnego czyszczenia.
- **Determinizm budowy.** Budżet `Update` liczy kroki pracy, nie czas ani klastry, i ten sam ciąg
  wywołań daje ten sam graf i postęp na każdej maszynie. Zegar nie wchodzi w decyzje budowy, bo
  w gramie gotowość hierarchii rozstrzyga, czy trasa się wspina. Czas mierzy tylko hak
  benchmarków.
- Każda kolejność budów daje ten sam graf co do bitu. Testy porównują przebudowę z budową od zera.
- **Przebudowa sięga tak daleko jak zmiana.** `Changed` oznacza tylko poziom 1. Wyżej klaster
  porównuje się z sobą sprzed budowy: inne klucze, klasy lub bliźniaki dają `rebuild` rodzica,
  inna sama macierz kosztów `refill`, a wszystko to samo zostawia rodzica nietkniętego.
- **Etykiety spójności** zostają, gdy interfejs spójności klastra się nie zmienił. Inaczej idzie
  przenazwanie lokalne, a gdy komponent mógł się rozpaść, całość w krokach. Póki trwa całość,
  `Ready` jest fałszem, a `Reaches` mówi „tak”.

## Pułapki

- Koszt ujemny psuje kopiec. Jest poza kontraktem i nic go nie sprawdza.
- `Backward` musi być lustrem `Forward`. Przejścia `SearchVia` muszą stepować i wyceniać jak graf
  budujący, w tej samej kolejności.
- Stan, którego `Layout.At` nie wymienia, na przykład wirtualny cel, stepuje tylko w swoim
  klastrze. Krok przez granicę z takiego stanu nie istnieje w grafie abstrakcyjnym, więc `NoPath`
  i etykiety by skłamały.
- `Changed` woła się po zmianie grafu, dla każdego stanu, którego kroki albo kroki do niego się
  zmieniły. `Changed` i `Update` tylko między zapytaniami, a `Search` jeden na gorutynę.
- Klaster wyższego poziomu trwa wiele wywołań `Update`. Na gotowość czeka się po `left == 0` albo
  `Ready`, nie po `built == 0`.
- Słownik dokładnego silnika to Size × 5 B, robiony przy pierwszym dokładnym szukaniu, osobno w
  każdym `Search`.
- Koszty w hierarchii to float32, zgodne z A* do 1e-4. Golden tras hierarchii nie ma, bo
  generator map zlewa się na arm64 w multiply-add.
- Przykład godoc nazwany identyfikatorem, którego pakiet nie ma, go/doc po cichu pomija, a vet
  go przepuszcza.

## Odrzucone

- Kopiec 4-arny: skalarny jest 1–2% wolniejszy, z SIMD szybszy o 10% na siatce, ale zmienia,
  która z równych ścieżek wychodzi, i tłum w gramie przeplanowuje o 55% więcej.
- Ponowne wypełnianie tylko wierszy dotkniętych zmianą: wypełnienie od wejścia osiedla prawie
  cały klaster, więc prawie każdy wiersz go dotyka.

## Pomiar

- Jeden przebieg na stronę, cały pomiar kilka minut, maszyna bezczynna, `-count=1 -cpu=1`, bo
  AVX2 obniża zegar. Binaria testów kompiluje się przed pomiarem (`go test -c`).
- Dwa drzewa porównuje program jednorazowy poza repo, zbudowany dwa razy z `replace` na każde
  drzewo: transkrypt ścieżek, `Stats` i postępu `Update` ma się zgadzać co do bajtu.
- Starą wersję mierzy osobny program z wersją z proxy, bo jeden build nie pomieści dwóch wersji
  jednej ścieżki modułu. Tę samą mapę i parę potwierdzają te same rozwinięcia, koszt i długość
  A*.
- Testy 4096² włącza `HPASTAR_SCALE=4096`. `GOEXPERIMENT` bierze się z Makefile, nie z powłoki.
  Pod `/home/karol/CODE` rządzi `go.work`, więc moduł mierzy się z `GOWORK=off`.
- `-bench` dopasowuje wzorzec poziom po poziomie po `/`; `|` działa tylko w nawiasach jednego
  poziomu.

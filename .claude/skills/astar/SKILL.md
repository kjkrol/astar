---
name: astar
description: Model solvera astar (github.com/kjkrol/astar) — węzły w tablicach po id, kopiec binarny kluczy i id z sitem z dziurą, jeden słownik ze stemplami, Wipe — kontrakt A*, testy i golden ścieżek, benchmarki, protokół pomiaru, co próbowano z SIMD i czemu odpadło, pułapki. Czytaj przy zmianie solvera, kopca, słowników, benchmarków lub dokumentacji, i z grama (plugins/navigation) przy pytaniu, co planner dostaje od solvera.
---

# astar

Generyczny solver A* (`Solver[T]`): `New(heuristic, opts...)`, `Solve`/`SolveInto`,
`Iter`/`Result`/`ResultInto`, `Wipe`; `Transitions` wołającego wypełnia bufor następnikami
(`Transition{To, Cost}`), `Heuristic` szacuje resztę. Jedyny użytkownik w workspace: gram,
`plugins/navigation/internal/pathfind/finder.go` (dwa solvery: po `cell.ID` i po węźle z poziomem,
`WithIndexedSliceDict`, `WithSuccessorCapacity`, `SolveInto`, `Wipe`; krawędź zerowa do
wirtualnego celu w `leveledFrom`).

## Model

- **Węzły w tablicach** (`astar.go`): `nodes []node[T]{ID, G, Parent int32}` po `int32`,
  `Parent` −1 na starcie, `current` id lub −1; `alloc` dokłada na koniec i rośnie podwajaniem
  od `WithInitCapacity`. Nic w szukaniu nie jest wskaźnikiem, więc GC nie skanuje węzłów typu
  bez wskaźników. **Pułapka**: `alloc` może przenieść `nodes`; pola bieżącego węzła kopiuje się
  do zmiennych przed pętlą następników, nigdy `&nodes[i]` przez `alloc`.
- **Kopiec** (`heap.go`): `openHeap{keys, nodes, pos}`, klucz F osobno od węzła, `pos` po id
  (−1 poza kopcem) dla `fix`. Binarny; sito niesie węzeł jako dziurę i pisze raz, gdzie
  wyląduje, ale porównuje dokładnie tak jak `container/heap` (`up` przerywa na `i == j ||
  !(key < keys[i])`, `down` bierze prawe dziecko tylko gdy ściśle mniejsze), więc remisy i
  ścieżki są te co v1.1.3 (`TestHeap_PopsAsContainerHeap` pinuje to lustrem).
- **SIMD, próbowane i odrzucone** (2026-10-28): kopiec 4-arny z wyborem najmniejszego z czterech
  dzieci jedną instrukcją (`LoadFloat64x4Array`, dwa shuffle AVX z `Min`, `LessEqual(m).ToBits()`,
  `TrailingZeros8`) dał −10% na siatce ośmiokierunkowej w buildzie SIMD, ale zmienia, która z
  kilku najtańszych ścieżek wychodzi, i w zatłoczonym ticku grama (1 000 piechurów przez jedną
  lukę) jednostki przeplanowują 55% więcej; skalarny 4-arny sam jest 1–2% wolniejszy. Pliki
  próby leżą poza repo (scratchpad sesji); liczby w `BENCHMARKS.md`. Nie wracać bez nowego
  powodu: arytmetyka następników (callback heurystyki na następnik) i porównania słownikowe
  (brak gather) też nie.
- **Słownik** (`node_dict.go`): jeden na stan, wartość to id węzła otwartego albo `^id`
  zamkniętego; `indexedSliceDict` to `ids []int32` + `stamps []uint8` (5 B slot), `clear`
  przesuwa stempel, co 255 szukań wymazuje; `Wipe(within)` robi to wcześniej między
  szukaniami; mapy (`mapDict`, `indexedMapDict`) czyszczą się `clear(m)`.
- **Pętla** (`next`, `expandCurrent`): test celu przy zdjęciu; `existing.G <= G` nie jest
  poprawą (pierwszy rodzic zostaje); stan zamknięty osiągnięty taniej otwiera się jako świeży
  węzeł (stary zostaje dla tych, których przez niego osiągnięto); `prev == from` na starcie;
  bufor przejść rośnie i zostaje.

## Kontrakt (doc.go, README „The contract")

Koszt dodaje się do ścieżki i nigdy nie jest ujemny; heurystyka nigdy nie szacuje więcej niż
najtańsza reszta (inaczej ścieżka nie jest najtańsza); spójna nie rozwija stanu dwa razy;
krok odmówiony to przejście niewygenerowane. Weighted A* to świadoma zamiana najtańszej ścieżki
na szybciej znalezioną. Skala udziałów [0, 1] (1 pełne tempo, 0 brak kroku, koszt = długość /
udział) jest wołającego (gram `board/cell`), solver widzi tylko koszty.

## Testy, golden, benchmarki

- `make test` zwykły build, `make test-simd` z eksperymentem; CI oba (Go 1.25 i 1.27).
- `testdata/paths.golden` pinuje ścieżki sześciu scen (`golden_test.go`, `-update` przepisuje).
  Zmiana, która ma zachować ścieżki, zachowuje golden; zmieniająca remisy przepisuje go i mówi
  dlaczego — i wtedy `routes.golden` grama też się zmieni.
- `optimal_test.go`: najtańsza ścieżka przeciw Dijkstrze, ponowne otwarcie, kroki zerowe,
  `prev`, przerwany `Iter`, zakres Indexera; `heap_test.go` kopiec przeciw `container/heap`
  na losowych operacjach z remisami.
- `make bench` (bez testów), `make bench-save` (×5 do `bench_results/`), `bench-simd`,
  `bench-save-simd`. Rungi: `Solver_Solve` (waga 1.0 pełne szukanie; 0.5 i 0.1 to reżimy
  niedopuszczalne, mierzą reset i krótkie szukanie), `Solver_SolveInto` (bufor wielokrotny,
  0 alokacji), `Solver_Octile` (osiem kierunków, kształt grama).

## Pomiar

Kopia bazowa drzewa w scratchpadzie, drzewo i baza na zmianę, maszyna bezczynna, ten sam build
po obu stronach, `-count=2` × 3 rundy, `-benchtime=10x`, `-cpu=1`; `compare.py` z grama
(`/home/karol/CODE/gram/.claude/skills/gram-perf/compare.py`) daje min i medianę. Go dopasowuje
`-bench` poziom po poziomie: benchmark płytszy niż wzorzec nie biegnie, więc `Solve` (cztery
poziomy) i `SolveInto|Octile` (trzy) mierzy się osobnymi wzorcami. Zysk liczy się od 3% mediany
w swoim reżimie bez straty w innym; odrzuceni kandydaci z liczbami w `BENCHMARKS.md`.

## Pułapki

- Zmiana sita, która zachowuje decyzje `container/heap` (ta sama kolejność porównań), trzyma
  golden; zmiana arności go zmienia, a z nim `routes.golden` grama i dynamikę jego tłumów.
- Koszt ujemny psuje kopiec (`fix` węzła poza kopcem) — poza kontraktem, nie sprawdzany.
- `GOEXPERIMENT` z Makefile, nie z powłoki: goły `go test` to build zwykły. AVX2 obniża zegar:
  porównania `-cpu=1`. `Float64x4.Permute` to AVX-512 (nie na i5-8265U); shuffle AVX to
  `ConcatPermute128Scalars` i `ConcatPermuteScalarsGrouped`.
- Indexer poza `[0, maxSize)` panikuje na indeksie slice'a — tak ma być (test).

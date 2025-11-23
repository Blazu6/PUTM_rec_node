## 1. Architektura: Maszyna Stanów (State Machine)

Logika węzła `LapTimer` opiera się na deterministycznej **Skończonej Maszynie Stanów (FSM)**. Takie podejście gwarantuje, że program wykonuje tylko obliczenia odpowiednie dla danej fazy wyścigu i eliminuje błędy logiczne (np. próba obliczenia delty przed ukończeniem pierwszego okrążenia).

Stan programu jest przechowywany w zmiennej `m_state` typu `enum class State`.

### Diagram Przepływu

stateDiagram-v2
    [*] --> WAITING
    
    state "WAITING_FOR_START" as WAITING
    note right of WAITING
        Out Lap / Wyjazd z boksu
    end note

    state "RECORDING_REFERENCE_LAP" as RECORDING
    note right of RECORDING
        Okrążenie 1: Budowanie mapy
    end note

    state "LAPPING" as LAPPING
    note right of LAPPING
        Okrążenie 2+: Wyścig i Delta
    end note

    WAITING --> RECORDING: Pierwsze przecięcie linii Start/Meta
    RECORDING --> LAPPING: Ukończenie pierwszego okrążenia
    LAPPING --> LAPPING: Kolejne przecięcia linii

### Opis Stanów

#### 1. `WAITING_FOR_START` (Oczekiwanie)
* **Kontekst:** Bolid wyjeżdża z garażu lub stoi na polach startowych (Out Lap).
* **Zachowanie:** System nasłuchuje pozycji GPS wyłącznie w celu wykrycia linii startu.
* **Ograniczenia:** Nie są zapisywane żadne dane o trasie, czas okrążenia nie jest liczony. Zapobiega to stworzeniu błędnej mapy referencyjnej na podstawie powolnego wyjazdu z boksu.

#### 2. `RECORDING_REFERENCE_LAP` (Nagrywanie Referencji)
* **Kontekst:** Pierwsze pełne, szybkie okrążenie pomiarowe.
* **Zachowanie:**
    * System rejestruje współrzędne GPS oraz czas przejazdu co 0.5 metra, tworząc wektor `m_reference_lap_sectors`.
    * Liczony jest czas bieżącego okrążenia.
* **Ograniczenia:** Wartość `Delta` wynosi 0.0, ponieważ nie istnieje jeszcze okrążenie, do którego można by się odnieść.

#### 3. `LAPPING` (Tryb Wyścigowy)
* **Kontekst:** Wszystkie kolejne okrążenia aż do zatrzymania programu.
* **Zachowanie:**
    * System porównuje obecną pozycję z zapisaną mapą (`m_best_lap_sectors`).
    * **Obliczana jest Delta:** Różnica czasu między obecnym przejazdem a najlepszym okrążeniem w tym samym punkcie toru.
    * Jeśli aktualne okrążenie okaże się szybsze od najlepszego, po przekroczeniu mety staje się ono nowym rekordem (nową referencją).

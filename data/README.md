# Benchmark Data

Place Cordeau & Laporte DARP benchmark instance files (`.txt`) in this directory.

## Standard Benchmark Sets

### Cordeau & Laporte (2003) — Type A and B

The canonical DARP benchmark set. Two families:

| Set | Time windows | Instances |
|-----|-------------|-----------|
| A   | Tight        | a2-16, a3-18, a3-24, a4-16, a4-24, a4-32, a5-40, a5-48, a6-48, a6-60 |
| B   | Loose        | b2-16, b3-18, b3-24, b4-16, b4-24, b4-32, b5-40, b5-48, b6-48, b6-60 |

The naming convention is `<set><K>-<2n>`:
- Set = A or B
- K = number of vehicles
- 2n = total number of customers (pickup + dropoff), so n = request pairs

### File Format

Each file follows this layout:

```
K  n  T  Q  L
0  x  y  d  e  l  q        ← origin depot (node 0)
1  x  y  d  e  l  q        ← pickup 1
...
n  x  y  d  e  l  q        ← pickup n
n+1  x  y  d  e  l  q     ← dropoff 1 (matches pickup 1)
...
2n  x  y  d  e  l  q      ← dropoff n
2n+1  x  y  d  e  l  q   ← return depot (node 2n+1)
```

Field meanings:
- `K`: number of vehicles
- `n`: number of request pairs (each request = one pickup + one dropoff)
- `T`: maximum route duration
- `Q`: vehicle capacity (maximum simultaneous load)
- `L`: maximum ride time per passenger
- Per node: `id  x  y  d  e  l  q`
  - `d`: service duration at the node
  - `e`, `l`: time window [earliest start, latest start]
  - `q`: load (+q for pickup, -q for dropoff, 0 for depot)

## Downloading the Instances

The Cordeau & Laporte instances are available from the SINTEF Applied Mathematics
benchmark library:

```
https://www.sintef.no/projectweb/top/darp/
```

Download the `.txt` files and place them in this directory.
The benchmark runner (`examples/run_benchmarks.jl`) will automatically discover all
`.txt` files here.

## Known Optimal Values

Selected reference objective values from Cordeau (2006) for validation:

| Instance | Best known |
|----------|-----------|
| a2-16   | 294.25    |
| a3-18   | 310.30    |
| a3-24   | 430.89    |
| a4-16   | 270.10    |
| a4-24   | 385.55    |

Note: The `test/fixtures/a2-16.txt` file committed to this repository is a **synthetic**
instance used for CI. For research comparisons use the official Cordeau instances.

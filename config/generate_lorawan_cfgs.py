import re
from pathlib import Path

# === Percorso file sorgente ===
input_file = Path("joao_headers.txt")

# === Percorso cartella di output ===
output_dir = Path("sd_cards")
output_dir.mkdir(exist_ok=True)

# === Chiavi costanti ===
APP_EUI = "0000000000000000"
APP_KEY = "A67AF941696B48269B4A535CDC6E2516"
NWK_KEY = "A67AF941696B48269B4A535CDC6E2516"

# === Estrazione dei devEUI dal file sorgente ===
with open(input_file, "r") as f:
    content = f.read()

# Trova tutti i blocchi devEUI
matches = re.findall(r"Collars_(\d+).*?devEUI\[\]\s*=\s*\{([^}]+)\}", content, re.DOTALL)

if not matches:
    print("❌ Nessun devEUI trovato nel file.")
    exit(1)

for num, devEUI_raw in matches:
    # Pulisce la stringa e compatta i byte esadecimali
    bytes_list = [b.strip().replace("0x", "") for b in devEUI_raw.split(",")]
    devEUI = "".join(bytes_list).upper()

    # Crea il contenuto del file di configurazione
    cfg_content = f"""# LoRaWAN configuration file (Cicerone Collar {num})
# Lines starting with # are ignored

devEUI={devEUI}
appEUI={APP_EUI}
appKEY={APP_KEY}
nwkKEY={NWK_KEY}
"""

    # Salva su file
    output_path = output_dir / f"lorawan_{int(num):02d}.cfg"
    with open(output_path, "w") as out:
        out.write(cfg_content)

    print(f"✅ Creato: {output_path.name}")

print(f"\nTutti i file salvati in: {output_dir.resolve()}")

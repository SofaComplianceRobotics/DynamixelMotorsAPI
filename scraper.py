"""
scrape_dynamixel.py
-------------------
Scrapes all Dynamixel motor pages from emanual.robotis.com and outputs
a JSON file with register addresses, sizes, and specs for each model.

Usage:
    pip install requests beautifulsoup4
    python scrape_dynamixel.py

Output:
    dynamixel_configs.json
"""

import re
import json
import time
import logging
from dataclasses import dataclass, asdict, field, make_dataclass, fields
from typing import Optional

import requests
from bs4 import BeautifulSoup

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

BASE_URL  = "https://emanual.robotis.com"
INDEX_URL = f"{BASE_URL}/docs/en/dxl/"

SESSION = requests.Session()
SESSION.headers.update({
    "User-Agent": "Mozilla/5.0 (compatible; DynamixelScraper/1.0)"
})


# ── Data model ─────────────────────────────────────────────────────────────────
from dynamixelmotorsapi._dynamixelmotorsconfigs import ModelConfig
# We extend the base ModelConfig with an 'errors' field to capture any issues during scraping.
MotorConfig  = make_dataclass(
    "ModelConfig",
    [(f.name, Optional[f.type], field(default=None)) for f in fields(ModelConfig)] +
    [("errors", list, field(default_factory=list))]
)

# ── Register name → field mapping ─────────────────────────────────────────────
#
# Each entry: (fragment_to_match, addr_field, len_field_or_None)
# Matching is case-insensitive, first-match wins.
# More specific entries must come before more generic ones.

REGISTER_MAP = [
    # Baud rate
    ("baud rate",              "addr_baud_rate",          "len_baud_rate",          "initial_baud_rate"),
    
    # Torque / mode
    ("torque enable",          "addr_torque_enable",       "len_torque_enable",       "initial_torque_enable"),
    ("operating mode",         "addr_operating_mode",      "len_operating_mode",      "initial_operating_mode"),

    # Position 
    ("goal position",          "addr_goal_position",       "len_goal_position",       "initial_goal_position"),
    ("present position",       "addr_present_position",    "len_present_position",    "initial_present_position"),
    ("position trajectory",    "addr_position_trajectory", "len_position_trajectory", "initial_position_trajectory"),
    ("position p gain",        "addr_position_p_gain",     "len_position_p_gain",     "initial_position_p_gain"),
    ("position i gain",        "addr_position_i_gain",     "len_position_i_gain",     "initial_position_i_gain"),
    ("position d gain",        "addr_position_d_gain",     "len_position_d_gain",     "initial_position_d_gain"),
    ("min position limit",     "addr_min_position",        "len_min_position",        "min_position_value"),
    ("max position limit",     "addr_max_position",        "len_max_position",        "max_position_value"),

    # Current
    ("present current",        "addr_present_current",     "len_present_current",     "initial_present_current"),

    # Velocity
    ("goal velocity",          "addr_goal_velocity",       "len_goal_velocity",       "initial_goal_velocity"),
    ("present velocity",       "addr_present_velocity",    "len_present_velocity",    "initial_present_velocity"),
    ("velocity trajectory",    "addr_velocity_trajectory", "len_velocity_trajectory", "initial_velocity_trajectory"),
    ("profile velocity",       "addr_velocity_profile",    "len_velocity_profile",    "initial_velocity_profile"),
    ("goal acceleration",      "addr_velocity_profile",    "len_velocity_profile",    "initial_velocity_profile"),   # PRO series uses same field for acceleration

    # Moving
    ("moving status",          "addr_moving_status",       "len_moving_status",       "initial_moving_status"),
    ("moving",                 "addr_moving",              "len_moving",              "initial_moving"),
]


# ── Series detection ───────────────────────────────────────────────────────────

def detect_series(url: str) -> str:
    u = url.lower()
    if "/dxl/y/"   in u: return "Y_SERIES"
    if "/dxl/p/"   in u: return "P_SERIES"
    if "/dxl/x/"   in u: return "X_SERIES"
    if "/dxl/mx/"  in u: return "MX_SERIES"
    if "/dxl/ax/"  in u: return "AX_SERIES"
    if "/dxl/rx/"  in u: return "RX_SERIES"
    if "/dxl/ex/"  in u: return "EX_SERIES"
    if "/dxl/dx/"  in u: return "DX_SERIES"
    if "/dxl/pro/" in u:
        # URLs ending in 'a/' or 'ra/' are the (A) advanced firmware
        return "PRO_A_SERIES" if re.search(r"[/-]r?a/?$", u) else "PRO_SERIES"
    return "UNKNOWN"


# ── Page fetching ──────────────────────────────────────────────────────────────

def fetch(url: str, retries: int = 3) -> Optional[BeautifulSoup]:
    for attempt in range(retries):
        try:
            resp = SESSION.get(url, timeout=15)
            resp.raise_for_status()
            soup = BeautifulSoup(resp.content, "html.parser")
            # save soup into a file for debugging
            with open("debug_soup.html", "w", encoding="utf-8") as f:
                f.write(str(soup))
            return soup
        except requests.RequestException as e:
            wait = 2 ** attempt
            log.warning(f"  Fetch error ({e}), retrying in {wait}s…")
            time.sleep(wait)
    return None


# ── Index scraping ─────────────────────────────────────────────────────────────

# URL path segments that are NOT individual motor pages
_SKIP_PATHS = {
    "/docs/en/dxl/",
    "/docs/en/dxl/protocol1/",
    "/docs/en/dxl/protocol2/",
    "/docs/en/dxl/dxl-quick-start-guide/",
    "/docs/en/dxl/x/",
    "/docs/en/dxl/mx/",
    "/docs/en/dxl/p/",
    "/docs/en/dxl/pro/",
    "/docs/en/dxl/y/",
}

def get_motor_urls() -> list[tuple[str, str]]:
    """
    Return a list of (model_name, absolute_url) for every motor listed
    in the sidebar of the DYNAMIXEL index page, restricted to /docs/en/dxl/.
    """
    soup = fetch(INDEX_URL)
    if not soup:
        raise RuntimeError("Could not fetch index page")

    seen  = set()
    motors = []

    # The sidebar <nav> contains ALL motor links in structured <li> elements.
    # We look for every <a> whose href starts with /docs/en/dxl/ and
    # is not one of the section index URLs.
    for a in soup.find_all("a", href=True):
        href = a["href"]

        # Normalise: strip fragments, ensure it starts with /docs/en/dxl/
        href = href.split("#")[0].rstrip("/") + "/"
        if not href.startswith("/docs/en/dxl/"):
            continue
        if href in _SKIP_PATHS:
            continue
        # Must be at least two levels deep (series/model/)
        parts = [p for p in href.split("/") if p]
        if len(parts) < 5:   # ['docs','en','dxl','<series>','<model>']
            continue

        url = BASE_URL + href
        if url in seen:
            continue
        seen.add(url)

        model = a.get_text(strip=True) or parts[-1].upper()
        motors.append((model, url))

    log.info(f"Found {len(motors)} motor pages in index")
    return motors


# ── Per-page parsing ───────────────────────────────────────────────────────────

def _clean_int(text: str) -> Optional[int]:
    """Parse a cell that may contain commas, spaces, negative signs."""
    text = re.sub(r"[,\s]", "", text.strip())
    # Grab the first number including optional leading minus
    m = re.search(r"-?\d+", text)
    return int(m.group()) if m else None


def parse_specs(soup: BeautifulSoup) -> dict:
    """
    Extract the Specifications table (the one with Item/Specifications columns).
    Returns a dict of lower-case item name → raw cell text.
    """
    specs = {}
    for table in soup.find_all("table"):
        headers = [th.get_text(strip=True).lower() for th in table.find_all("th")]
        if "item" in headers and ("specifications" in headers or "specification" in headers):
            for row in table.find_all("tr")[1:]:
                cells = row.find_all("td")
                if len(cells) >= 2:
                    key = cells[0].get_text(strip=True).lower()
                    val = cells[1].get_text(strip=True)
                    specs[key] = val
    return specs


def parse_resolution(specs: dict) -> Optional[int]:
    """Extract encoder resolution (pulse/rev) from the specs dict."""
    for key, val in specs.items():
        if "resolution" in key:
            m = re.search(r"([\d,]+)\s*\[?pulse", val, re.IGNORECASE)
            if m:
                return int(m.group(1).replace(",", ""))
            # Fallback: first standalone large number
            m = re.search(r"\b([\d,]{3,})\b", val)
            if m:
                n = int(m.group(1).replace(",", ""))
                if 256 <= n <= 10_000_000:
                    return n
    return None


def parse_control_tables(soup: BeautifulSoup) -> dict[str, dict]:
    """
    Parse ALL control tables on the page (EEPROM + RAM).
    Returns flat dict: lowercase_name → {address: int, size: int, initial: str|None}
    """
    registers = {}
    tables = soup.find_all("table")
    tables = list(filter(lambda table: table.find(string="Address") and table.find(string="Data Name"),
                    tables))    

    for table in tables:
        # Identify column indices from header row
        header_row = table.find("tr")
        if not header_row:
            continue
        headers = [th.get_text(strip=True).lower() for th in header_row.find_all(["th", "td"])]

        if "address" not in headers and "data name" not in headers and "size(byte)" not in headers:
            continue

        address_index = headers.index("address")
        size_index = headers.index("size(byte)")
        name_index = headers.index("data name")

        # Initial value column is optional
        initial_value_index = headers.index([i for i in headers if re.match(r"initial.*value", i)][0])

        for row in table.find_all("tr")[1:]:
            cells = row.find_all("td")
            if len(cells) <= max(address_index, size_index, name_index):
                continue

            addr = _clean_int(cells[address_index].get_text())
            size = _clean_int(cells[size_index].get_text())
            if addr is None or size is None:
                continue   # skip separator / ellipsis rows

            # Strip footnote markers, parenthetical address hints, anchor tags
            raw_name = cells[name_index].get_text(separator=" ", strip=True)
            name = re.sub(r"\([\d,\s~]+\)", "", raw_name)   # remove "(596)"
            name = re.sub(r"\s+", " ", name).strip().lower()
            if not name:
                continue
            
            initial = _clean_int(str(cells[initial_value_index].next_element)) if (initial_value_index and initial_value_index < len(cells)) else None
            registers[name] = {"address": addr, "size": size, "initial": initial}

    return registers


def parse_current_unit(soup: BeautifulSoup) -> Optional[float]:
    """
    Extract the current unit (mA per unit) from the page, in the Current Limit section
    """
    # get table from "Current Limit"section
    table = soup.css.select_one("h3[id*='current-limit'] ~ table")
    if table:
        if table.find(string=re.compile(r"Unit", re.IGNORECASE)):
            # find the cell that contains "Unit" and extract the number from it
            unit_cell_index = None
            for i, cell in enumerate(table.find_all("th")):
                if re.search(r"Unit", cell.get_text(), re.IGNORECASE):
                    unit_cell_index = i
                    break
            unit__cell = table.find_all("td")[unit_cell_index] if unit_cell_index is not None else None
            if unit__cell:
                text = unit__cell.get_text(strip=True)
                m = re.search(r"([\d,.]+)\s*\[?m?A\]?", text, re.IGNORECASE)
                if m:
                    converted = float(m.group(1).replace(",", ""))
                    if re.search(r"\bA\b", text, re.IGNORECASE): # if in A, convert to mA
                        converted *= 1000
                    return converted
        


def map_registers(registers: dict) -> dict:
    """Apply REGISTER_MAP to produce ModelConfig-compatible field dict."""
    result = {}

    print("Extracted register: \n", registers.keys())
    # print(REGISTER_MAP)

    for reg_name, reg_data in registers.items():
        for fragment, addr_field, len_field, ini_field in REGISTER_MAP:
            if fragment in reg_name:
                if addr_field not in result:
                    result[addr_field] = reg_data["address"]
                    if len_field:
                        result[len_field] = reg_data["size"]
                        if ini_field:
                            result[ini_field] = reg_data["initial"]
                break

    print(result)
    return result


# ── Main per-motor scrape ──────────────────────────────────────────────────────

def scrape_motor(model: str, url: str) -> MotorConfig:
    config = MotorConfig(model=model, series=detect_series(url), url=url)

    soup = fetch(url)
    if not soup:
        config.errors.append("Failed to fetch page")
        return config
    

    # 1. Specifications
    specs = parse_specs(soup)
    config.resolution = parse_resolution(specs)

    # 2. Control tables
    registers = parse_control_tables(soup)
    if not registers:
        config.errors.append("No control table found")
        return config

    # 3. Map registers → fields
    mapped = map_registers(registers)
    for field_name, value in mapped.items():
        if hasattr(config, field_name):
            setattr(config, field_name, value)

    # 4. Get current unit
    config.current_unit = parse_current_unit(soup)
    

    return config


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    motors = get_motor_urls()

    results    = []
    errors     = []
    total      = len(motors)

    for i, (model, url) in enumerate(motors, 1):
        log.info(f"[{i:3d}/{total}] {model:30s}  {url}")
        config = scrape_motor(model, url)

        if config.errors:
            log.warning(f"  ⚠  {config.errors}")
            errors.append({"model": model, "url": url, "errors": config.errors})

        results.append(asdict(config))
        time.sleep(0.3)   # polite delay between requests

    # Write output
    out_path = "./dynamixelmotorsapi/dynamixel_configs.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    log.info(f"\n✓  Wrote {len(results)} configs to {out_path}")
    if errors:
        log.warning(f"   {len(errors)} pages had errors:")
        for e in errors:
            log.warning(f"     {e['model']}: {e['errors']}")


if __name__ == "__main__":
    main()
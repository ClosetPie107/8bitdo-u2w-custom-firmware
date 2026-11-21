#!/usr/bin/env python3
"""Compile and test the C CSA #2 channel selector in patches/csa2.c against BLE 5.0 vectors."""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

PATCHES = Path(__file__).resolve().parent

HARNESS_C = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "s2_state.h"

s2_state_t g_s2_test_state;
static uint8_t g_remap[38];

const uint8_t *host_csa2_remap_table(void) {
    return g_remap;
}

static uint16_t py_perm(uint16_t v) {
    v = (uint16_t)(((v & 0xAAAAu) >> 1) | ((v & 0x5555u) << 1));
    v = (uint16_t)(((v & 0xCCCCu) >> 2) | ((v & 0x3333u) << 2));
    v = (uint16_t)(((v & 0xF0F0u) >> 4) | ((v & 0x0F0Fu) << 4));
    return v;
}

static uint16_t py_mam(uint16_t a, uint16_t b) {
    return (uint16_t)((17u * (uint32_t)a + (uint32_t)b) & 0xFFFFu);
}

static uint8_t ref_csa2(const uint8_t chn_map[5], uint16_t counter, uint16_t chan_id) {
    uint16_t x = (uint16_t)(counter ^ chan_id);
    x = py_mam(py_perm(x), chan_id);
    x = py_mam(py_perm(x), chan_id);
    x = py_mam(py_perm(x), chan_id);
    uint16_t prn_e = (uint16_t)(x ^ chan_id);
    uint8_t unmapped = (uint8_t)(prn_e % 37u);
    if (chn_map[unmapped >> 3] & (1u << (unmapped & 7u))) {
        return unmapped;
    }
    uint8_t used[37];
    uint8_t n = 0;
    for (uint8_t c = 0; c < 37u; c++) {
        if (chn_map[c >> 3] & (1u << (c & 7u))) used[n++] = c;
    }
    return used[((uint32_t)n * (uint32_t)prn_e) >> 16];
}

int main(void) {
    /* 1. BLE 5.0 Vol 6 Part C §3.1 Sample Data 1 (all 37 channels) */
    const uint8_t all37[5] = { 0xff, 0xff, 0xff, 0xff, 0x1f };
    const uint8_t exp1[4] = { 25, 20, 6, 21 };
    for (uint16_t c = 0; c < 4; c++) {
        assert(csa2_select_channel(all37, c, 0x305fu) == exp1[c]);
    }
    printf("PASS: C csa2_select_channel BLE 5.0 Sample 1 (all 37 ch)\n");

    /* 2. BLE 5.0 Vol 6 Part C §3.1 Sample Data 2 (9 used channels + remap) */
    const uint8_t ch9[5] = { 0x00, 0x06, 0xe0, 0x00, 0x1e };
    const uint8_t remap9[9] = { 9, 10, 21, 22, 23, 33, 34, 35, 36 };
    memset(g_remap, 0, sizeof(g_remap));
    memcpy(g_remap, remap9, sizeof(remap9));
    g_remap[37] = sizeof(remap9);
    const uint8_t exp2[4] = { 35, 9, 33, 21 };
    for (uint16_t c = 0; c < 4; c++) {
        assert(csa2_select_channel(ch9, c, 0x305fu) == exp2[c]);
    }
    printf("PASS: C csa2_select_channel BLE 5.0 Sample 2 (9 used ch)\n");

    /* 3. Exhaustive check over all 65536 counters against exact % 37 reference */
    for (uint32_t c = 0; c < 65536u; c++) {
        assert(csa2_select_channel(all37, (uint16_t)c, 0x305fu) == ref_csa2(all37, (uint16_t)c, 0x305fu));
        assert(csa2_select_channel(ch9, (uint16_t)c, 0x305fu) == ref_csa2(ch9, (uint16_t)c, 0x305fu));
    }
    printf("PASS: C csa2_select_channel matches reference for all 65536 counters (mapped & remapped)\n");

    /* 4. Symmetric/asymmetric/no-change PHY update resolver */
    assert(s2_resolve_phy_update(1u, 2u, 2u) == 2u);
    assert(s2_resolve_phy_update(2u, 1u, 1u) == 1u);
    assert(s2_resolve_phy_update(1u, 0u, 0u) == 0u);
    assert(s2_resolve_phy_update(2u, 2u, 2u) == 0u);
    assert(s2_resolve_phy_update(1u, 2u, 1u) == 0u);
    assert(s2_resolve_phy_update(1u, 1u, 2u) == 0u);
    assert(s2_resolve_phy_update(1u, 2u, 0u) == 0u);
    assert(s2_resolve_phy_update(1u, 0u, 2u) == 0u);
    printf("PASS: C s2_resolve_phy_update verified!\n");
    return 0;
}
"""


def main() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        harness = Path(tmp) / "test_csa2_harness.c"
        exe = Path(tmp) / "test_csa2"
        harness.write_text(HARNESS_C)
        subprocess.run(
            [
                "cc", "-std=gnu99", "-Wall", "-Wextra", "-Werror",
                "-DHOST_TEST", f"-I{PATCHES}",
                str(PATCHES / "csa2.c"), str(harness), "-o", str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()

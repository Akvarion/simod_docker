#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Thin executable wrapper for BT XML demo.

Note:
- Questo file non costruisce direttamente gli alberi BT.
- Il binding XML -> funzioni Action/Condition e l'assegnazione dei nomi
  dei tree (`Supervisor`, `SRM1`, `SRM2`) avviene in
  `bt_xml_demo/runner.py` dentro `main()`.
"""

from bt_xml_demo.runner import main


if __name__ == "__main__":
    main()

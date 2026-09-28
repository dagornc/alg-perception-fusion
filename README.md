# ALG_PERCEPTION_FUSION — portage Rust

Implémentation Rust de référence d'un filtre de perception multi-capteurs,
associée à une référence Python normative écrite en miroir.

Statut : **implémentation de référence simplifiée**. Ce dépôt ne revendique ni
validation opérationnelle, ni validation sur un essaim réel.

## Ce qui est implémenté

- état 2D `[px, py, vx, vy]` avec mouvement à vitesse quasi constante ;
- EKF réduit dont le modèle et les observations sont linéaires (il équivaut ici
  exactement à un filtre de Kalman) ;
- fusion locale séquentielle d'une mesure de vitesse d'odométrie et d'une
  mesure de position GNSS ;
- rejet d'outlier par seuil d'innovation à quatre écarts-types ;
- ajout périodique d'une piste de position Edge dans le scénario `edge` ;
- scénario `dropout` où la mesure GNSS n'arrive qu'un pas sur quatre ;
- métriques : RMSE de position, premier pas de convergence, fausses pistes,
  outliers rejetés et nombre de mesures acceptées ;
- générateur MT19937 compatible avec `random.Random` de CPython, copié du socle
  commun RUST-13 ;
- zéro dépendance Rust externe.

Les perturbations du simulateur sont uniformes et bornées afin de garantir une
simulation déterministe légère. Elles ne constituent pas une validation de
l'hypothèse de bruits gaussiens du modèle conceptuel.

## Utilisation

```sh
cargo run --release -- 1000 nominal 80
python3 reference/sim_perception_fusion.py 1000 nominal 80
```

Les deux programmes acceptent exactement trois arguments positionnels :
`seed`, `scenario` (`nominal`, `outliers`, `edge`, `dropout`) et `steps`. Ils
émettent le même en-tête et la même ligne CSV.

## Ce qui est vérifié

Mesures réalisées sur ce commit :

- `cargo build` : succès ;
- `cargo test` : **12 tests passés**, 0 échec (6 unitaires et 6 intégration) ;
- `cargo build --release` : succès ;
- `python3 verify_parite_rust.py` : **200 comparaisons**, soit 50 graines
  (`1000..1050`) × 4 scénarios × 80 pas ; **0 écart** ;
- statut : **parité bit-à-bit vérifiée sur les 200 comparaisons mesurées**.

Rejouer les vérifications :

```sh
cargo build
cargo test
cargo build --release
python3 verify_parite_rust.py
```

Le harnais compare la ligne CSV complète comme une chaîne, y compris la RMSE
formatée à 12 décimales et toutes les métriques entières.

## Ce qui ne l'est pas

- pas d'UKF, de filtre particulaire, d'attitude 3D ni de modèle capteur complet ;
- pas de piste multi-cible avec association de données ; la métrique de fausse
  piste mesure ici l'acceptation erronée d'un outlier injecté ;
- pas de fusion hiérarchique distribuée à 100 plateformes ou davantage ; le
  GAP-4 de la spécification reste ouvert ;
- pas de bruit gaussien, de données terrain, de vol réel, de HIL/SIL certifié,
  ni d'analyse de sûreté ;
- la parité mesurée porte sur la matrice de graines/scénarios indiquée, pas sur
  tous les paramètres possibles ;
- l'algorithme source est au statut conceptuel/idea et ce dépôt ne le marque
  pas `validated`.

## Licence

MIT — voir [LICENSE](LICENSE).

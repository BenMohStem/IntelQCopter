# IntelQCopter
## Cahier des charges système

**Révision 1.0 - 28 octobre 2026**

Plateforme F450 de recherche et d'apprentissage : commande embarquée, estimation inertielle, simulation et navigation autonome progressive.

**Référence : CDC-IQC-001. Statut : spécification de travail à examiner.**

Le présent document adopte une démarche d'ingénierie système de niveau universitaire avancé : exigences identifiées, interfaces explicites, critères de recette, traçabilité et gestion de l'incertitude. Il ne revendique aucune affiliation, certification ou approbation du MIT.

### Contrat de lecture

**D = décidé** par le porteur du projet ; **P = proposé** pour dimensionner les essais ; **TBD = à déterminer** par mesure ou décision. Une exigence D exprime une intention, jamais une preuve de réalisation. Les objectifs P ne deviennent contractuels qu'après revue. Les résultats historiques ne valent que pour leur source et leur configuration enregistrées.

La version prioritaire est l'ESP32-S3 avec manette Xbox Series X/S. L'architecture RP2350/RP2040 avec LoRa et CC1101 est conservée comme évolution distincte. L'ordinateur compagnon ROS 2 est optionnel. La réalisation du PCB et la construction des scènes de simulation restent sous la conduite du porteur du projet.

**Restriction actuelle : aucun armement physique ni signal moteur.** La recette présente porte sur le logiciel, les capteurs et les modèles ; elle n'autorise pas le vol.

### Livrables de cette révision

Document PDF pour revue, source Markdown modifiable et version HTML autonome. Le matériel existant, les gains, le firmware et les exports PCB ne sont pas modifiés par cette spécification.

---
## 1. Mission, périmètre et acteurs

### 1.1 Mission principale

Construire un quadricoptère F450 compréhensible, reproductible et testable, capable à terme de vol stabilisé commandé par manette. Le pilote commande le taux de lacet, les angles de roulis/tangage et la vitesse verticale ; le centrage vertical demande le maintien d'altitude. Le centrage roulis/tangage demande une assiette nulle, sans freinage horizontal ni maintien de position imposé.

**Une assiette nulle ne signifie pas une vitesse horizontale nulle.** Le drone peut continuer à dériver sous l'effet de son inertie et du vent. Ajouter ultérieurement un freinage demanderait une mesure ou estimation fiable de vitesse et un nouveau mode, explicitement approuvé.

### 1.2 Extension autonome

Un ordinateur compagnon optionnel reçoit une mission composée de n points, avec n supérieur ou égal à 2. Après validation opérateur, le premier point représente le départ et le dernier l'arrivée. Le compagnon planifie, suit la trajectoire et adapte ses consignes face aux obstacles détectés. Il transmet des commandes bornées au FC ; seul le FC commande les actionneurs.

### 1.3 Acteurs et responsabilités

| Acteur | Responsabilité |
|---|---|
| Porteur / opérateur | Confirmer le matériel, les repères et les conditions physiques ; approuver les critères et toute activation moteur. |
| Développement FC | Acquisition, calibration, estimation, commandes, budgets temps réel et diagnostics. |
| Développement compagnon | Mission, perception, planification, messages au FC et gestion de perte de liaison. |
| Validation | Conserver les originaux, reproduire les tests et distinguer modèle, émulation et matériel. |

### 1.4 Hors périmètre initial

GPS, vol longue distance, autonomie extérieure garantie, certification aéronautique, BMS par cellule, cartographie exhaustive et performances de compétition garanties. L'ajout de ces fonctions nécessite une nouvelle revue ; aucun de ces éléments n'est implicitement fourni par ROS 2 ou YOLO.

---
## 2. Configuration de référence et inconnues

| Élément | Référence / état |
|---|---|
| Structure | F450 / Q450 en X ; entraxe diagonal nominal 450 mm à mesurer sur l'assemblage. |
| Propulsion | Quatre moteurs annoncés 1400 KV ; modèle exact non confirmé ; hélices 9047 et quatre ESC annoncés 40 A. |
| Batterie | LiPo 3S, 4200 mAh, marquage 120C ; capacité utile, masse et courant admissible réels à caractériser. |
| FC prioritaire | ESP32-S3 DevKitC-1 ; variante mémoire et clone physique à identifier. |
| Capteurs | MPU6050, BMP280, module magnétomètre annoncé GY-271 et TOFSense F2 Mini. Identifier les puces réelles. |
| Radio pilote | Xbox Series X/S ; compatibilité directe ESP32-S3 / transport Bluetooth non validée. |
| Compagnon | Raspberry Pi 4/5 ou Jetson, caméra RGB et ROS 2 C++ ; matériel et OS non sélectionnés. |

### Paramètres fournis pour simulation

Le fichier utilisateur params.txt donne m = 1,038 kg, variance 0,000257 kg², soit un écart-type annoncé de 0,0160312 kg. Les inerties diagonales proposées sont Jxx = Jyy = 0,006421 kg·m² et Jzz = 0,011853 kg·m². Le rayon des moteurs est 0,225 m ; la projection en X vaut environ 0,1591 m.

Ces nombres sont des **entrées de modèle**, pas des mesures certifiées. Le fichier propose une batterie de 320 g, contrairement à l'ancienne estimation d'environ 150 g. Il contient aussi des hypothèses de covariance à réexaminer : des incertitudes indépendantes sur quatre masses ne s'additionnent pas comme quatre erreurs parfaitement corrélées. La masse ne constitue pas, seule, un bruit de processus Q.

### Inconnues bloquantes

TBD-01 : masse prête au vol et centre de masse. TBD-02 : courbe poussée/commande et régime moteur. TBD-03 : constante de temps moteur/ESC. TBD-04 : couple de réaction et sens de rotation. TBD-05 : inerties et produits d'inertie. TBD-06 : version des modules, tensions et adresses. TBD-07 : transport Xbox compatible. TBD-08 : orientation et usage du lidar. TBD-09 : courant maximal mesuré et capacité utile de la batterie.

**Point critique Xbox :** l'ESP32-S3 ne fournit pas le Bluetooth Classic de l'ESP32 original. Une bibliothèque de manette ne garantit donc pas sa compatibilité. Le transport BLE précis et la version du contrôleur doivent être démontrés ; à défaut, prévoir un récepteur externe compatible ou une passerelle, sans changer le cœur du FC.

---
## 3. Architecture et interfaces électriques

Deux domaines logiciels sont séparés : le FC assure la stabilité et reste autonome sans compagnon ; le compagnon produit des intentions de mouvement. Les données lentes passent par des échanges bornés et horodatés. Aucun accès I²C lent, traitement d'image ou écriture de journal ne doit bloquer la boucle critique.

| Interface | Affectation proposée / décidée |
|---|---|
| MPU6050 | SDA GPIO9 ; SCL GPIO8 ; VCC 3,3 V ; AD0 GND ; INT non câblé. |
| Magnétomètre | SDA vers MPU XDA ; SCL vers MPU XCL ; VCC 3,3 V ; DRDY non câblé. Choix de consolidation proposé. |
| BMP280 | SDA GPIO41 ; SCL GPIO40 ; VCC et CSB 3,3 V ; SDO GND, adresse 0x76. |
| Lidar | VCC régulé 5 V ; RX/SDA vers GPIO41 ; TX/SCL vers GPIO40 ; adresse I²C initiale 0x08. |
| ESC1 / 2 / 3 / 4 | Signaux GPIO4 / GPIO1 / GPIO14 / GPIO21 ; correspondance aux coins et sens moteur encore à valider. |
| Alimentation | Un seul BEC 5 V vérifié ; autres positifs isolés ; toutes les masses communes. Aucun courant moteur dans les pistes FC. |

IF-01 [D] : les signaux et résistances de rappel I²C sont à 3,3 V. Les alimentations sont vérifiées avant connexion. Le 3S brut ne doit jamais atteindre 5V/3V3 du DevKit. La coexistence USB/BEC doit éviter le retour d'alimentation.

IF-02 [D] : après initialisation complète du MPU, désactiver I2C_MST_EN et activer I2C_BYPASS_EN, en préservant les bits FIFO ; vérifier la relecture. Réappliquer après un reset du MPU. XDA/XCL ne sont ni un prolongateur de câble ni une connexion passive permanente.

IF-03 [D] : le lidar passe en I²C après configuration et sauvegarde vérifiées. Pour utiliser temporairement GPIO40/41 en UART, isoler le BMP et tout autre esclave de ce bus ; arrêter UART avant de démarrer I²C. Un adaptateur USB-TTL 3,3 V logique avec alimentation 5 V permet une configuration séparée.

IF-04 [P] : commencer le bus lent à 100 kHz ; toute augmentation, modification de pull-up ou longueur de câble exige une vérification électrique et une régression temporelle. Les plans et le firmware actuels ne prouvent pas encore cette nouvelle topologie.

---
## 4. Exigences fonctionnelles du FC

| ID / statut | Exigence | Vérification attendue |
|---|---|---|
| FC-01 D | Fonctionner sans ROS 2 ni ordinateur compagnon. | Démarrage et exécution sur banc, compagnon absent. |
| FC-02 D | Consigne de lacet en vitesse angulaire ; pas de cap absolu imposé par défaut. | Entrées positives/négatives et retour au neutre. |
| FC-03 D | Roulis et tangage en angle, avec boucles internes de vitesse angulaire. | Échelons limités et retour d'assiette au neutre. |
| FC-04 D | Manche vertical centré : maintien altitude ; déplacement : vitesse verticale signée. | Simulation avec consignes z/vz, sans vérité terrain cachée dans le FC. |
| FC-05 D | Aucun freinage XY ni maintien de position dans le mode stable initial. | Relâchement du manche : cible angle nulle, dérive possible. |
| FC-06 D | Profil de manches doux ; RT augmente, LT diminue la demande dans les limites. | Courbes entrée/sortie, saturations et neutralité. |
| FC-07 P | Plafond collectif boost de 1700 à 1800 µs à sélectionner, réserve pour corrections. | Mélangeur saturé : priorité attitude, anti-windup et limites moteur. |
| FC-08 D | Saturations, anti-windup et dérivée filtrée explicités ; activation Ki/Kd justifiée par essais. | Tests de saturation, retour au régime et bruit. |
| FC-09 D | Aucun signal moteur ni armement physique dans la révision actuelle. | Inspection source, mesure des broches ; aucun PWM/Servo actif. |

### Six correcteurs : architecture et décision restante

Le souhait de six PID correspond à deux pour le roulis, deux pour le tangage, un pour le lacet et un pour le vertical. Pour satisfaire simultanément maintien d'altitude et commande vz, une option est une loi externe z vers vz et un PID interne vz : la loi externe peut être proportionnelle, sans constituer un septième PID complet. Le choix exact reste à examiner ; le nombre « six » ne remplace pas une définition des états, unités et saturations.

### Loi de commande lisible

Pour chaque boucle : erreur e = consigne - mesure ; commande u = Kp·e + Ki·intégrale(e) + Kd·dérivée filtrée. Les unités de chaque gain, la période, la convention de signe et le domaine de validité doivent figurer dans la configuration. Aucun gain numérique ne sera présenté comme précis avant identification de la propulsion et validation du modèle.

---
## 5. Acquisition, calibration et estimation

| ID / statut | Exigence | Critère de recette |
|---|---|---|
| EST-01 D | Conserver la calibration multi-pose et le repère de montage documentés. | Coefficients, données sources et montage identifiables ; pas de refit sur validation. |
| EST-02 D | Restaurer les trims volatils du MPU au démarrage et vérifier leur relecture. | Démarrage, reset logiciel et coupure complète ; pas de double soustraction du biais. |
| EST-03 D | Inclure biais gyro, échelles et transformation capteur-vers-drone. | Rotations physiques signées sur X/Y/Z, positives et négatives. |
| EST-04 D | Estimation d'angle scalaire gyro/accéléromètre, bruit et limites explicités. | Rejeu indépendant et perturbations d'accélération ; pas d'assimilation automatique de l'accélération à la gravité. |
| EST-05 D | Filtre vertical à deux états z et vz ; sources et repères explicités. | Tests de prédiction/correction, délais, perte de capteur et covariance. |
| EST-06 D | Données invalides, répétées, périmées ou saturées identifiées. | Injection de fautes, délais et trames défectueuses. |
| EST-07 P | Capteurs lents traités à leur cadence réelle, avec âge de mesure. | Journal des timestamps et latences, absence de données artificiellement « fraîches ». |

### Modèle de calibration et statistiques

Une écriture générale est a_cor = C·(a_brut - b), où b contient les biais et C les facteurs d'échelle, couplages et rotation nécessaires. Si une partie de b est déjà appliquée dans les registres matériels, le logiciel ne retire que le résidu documenté. L'offset utilisateur du MPU n'est pas une mémoire permanente de calibration.

Pour N vecteurs x_i : moyenne mu = somme(x_i)/N ; covariance empirique S = somme[(x_i-mu)(x_i-mu)^T]/(N-1). Les termes hors diagonale décrivent les variations communes des axes. Une covariance nulle ne prouve pas l'indépendance sans hypothèse supplémentaire.

La covariance de la moyenne vaut S/N uniquement sous hypothèses adaptées d'indépendance et stationnarité. À 1 kHz, autocorrélation et filtrage peuvent réduire fortement le nombre d'échantillons indépendants. Les rapports doivent présenter covariance, dérive temporelle, température, bruit et incertitude de référence séparément ; aucune Gaussianité conjointe n'est supposée sans diagnostic.

Pour le filtre vertical : x = [z,vz] ; A = [[1,dt],[0,1]] ; B = [dt²/2,dt]^T pour l'accélération verticale. Les matrices Q et R doivent provenir d'un modèle de bruit discret et de mesures ; une variance de masse ne peut pas être copiée directement dans Q. Les innovations et leur cohérence sont contrôlées.

---
## 6. Temps réel et logiciel embarqué

| ID / statut | Exigence | Vérification attendue |
|---|---|---|
| RT-01 D | Cible de boucle FC 1000 Hz, période nominale 1 ms. | Horodatage acquisition, estimation, commande et publication ; charge complète représentative. |
| RT-02 P | Budget initial calcul critique <= 800 µs, réserve >= 200 µs. | Profilage matériel ; objectif non satisfait par simple débit moyen. |
| RT-03 P | Aucun dépassement observé sur 10 min sous charge nominale et injections définies. | Maximum, quantiles, jitter et compteurs ; ne constitue pas une preuve WCET. |
| RT-04 D | Communications et journaux bornés ; aucune attente infinie dans le chemin critique. | Timeouts, saturation files et retrait périphérique. |
| RT-05 D | Un main.cpp Arduino, style C autant que possible, fonctions courtes et accolades Allman. | Revue et compilation PlatformIO de la configuration épinglée. |
| RT-06 D | Acquisition capteur distinguée du traitement, du calcul PID et de la cadence ESC. | Traces de chaque étape ; protocole ESC validé séparément. |

### Répartition de charge

Le découpage en deux cœurs n'est adopté qu'après mesure et analyse des accès partagés. Un seul propriétaire gère chaque bus I²C ; les échantillons sont transmis par une file bornée ou un instantané cohérent. Les timestamps suivent une base monotone. Une file pleine produit un diagnostic et une stratégie explicite, jamais une corruption silencieuse.

Le 1 kHz demandé concerne le FC ; il n'impose pas de lire le baromètre, le lidar ou la caméra à 1 kHz. La sortie ESC dépend du protocole et de l'ESC exact ; « 40 A » ne spécifie ni sa fréquence d'entrée ni sa réponse dynamique.

### État réellement connu

Des enregistrements antérieurs ont observé environ 1000,69 échantillons/s et un maximum de handler de 822 µs pour une source identifiée. Ce résultat dépasse déjà le budget proposé de 800 µs ; il ne valide ni une boucle PID complète ni la nouvelle topologie I²C. La révision ultérieure de garde des broches a été compilée et testée sur hôte, sans nouvel upload attesté dans le dernier état enregistré.

Une compilation valide la construction ; des faux périphériques sur PC valident certains chemins logiciels ; aucune de ces preuves n'est une mesure de timing ESP32 ni un vol. Tout rapport doit nommer son moteur réel et ses interfaces simulées.

---
## 7. Manette, modes et ordinateur compagnon

| ID / statut | Exigence | Vérification attendue |
|---|---|---|
| HMI-01 D | Mode 2 : gauche lacet/vz, droite roulis/tangage ; RT boost, LT réduction. | Journal des entrées normalisées, deadzone et signes. |
| HMI-02 D | Bas du D-pad + A pendant 5 s : demande mode stable. | Durée continue ; relâchement remet le compteur à zéro ; un seul événement. |
| HMI-03 D | Bas du D-pad + B pendant 5 s : demande mode navigation. | Acceptée seulement si compagnon et mission valides ; refus expliqué sinon. |
| HMI-04 D | Bas du D-pad + LT pendant 2 s : demande désarmement. | Seuil LT à définir ; aucune activation accidentelle par bruit. |
| COM-01 D | FC autonome sans compagnon ; présence compagnon ne change pas spontanément de mode. | Démarrage, branchement, débranchement et redémarrage compagnon. |
| COM-02 P | Liaison FC-compagnon UART 3,3 V encadrée, brochage/débit TBD. | CRC, version, longueur, compteur, timestamp et validité des consignes. |
| COM-03 P | Heartbeat 10 Hz ; données périmées après 300 ms, valeur à revoir. | Retard, répétition, arrêt, mauvaise séquence ; état de dégradation enregistré. |

### Contrat de commande

Le message compagnon comporte version de protocole, identifiant de mission, compteur, mode demandé, unités SI, consignes bornées et durée de validité. Le FC renvoie état, mode accepté, âge des capteurs, défauts et saturation. Une trame invalide ne renouvelle pas la validité de la commande précédente. L'UART n'est pas une connexion électrique directe au RS232.

Le compagnon ne peut ni contourner l'interdiction d'armement ni écrire directement les moteurs. Le passage stable/navigation demande une transition sans saut de consigne, la remise à jour des intégrateurs concernés et une règle de priorité pilote explicite. Le comportement après perte de compagnon ou de manette reste à approuver : ne pas inventer un atterrissage automatique ou une coupure silencieuse.

### Poste sol ROS 2

L'utilisateur choisit n points sur une interface, examine leurs coordonnées et confirme l'envoi. Le poste transmet la mission au compagnon ; il n'exécute pas la boucle de navigation à sa place. Les repères, origine locale et transformation carte/odom/base_link sont explicites. L'implémentation ROS 2 garde une organisation C++ normale en paquets ; elle n'est pas contrainte à un seul fichier.

---
## 8. Navigation, perception et observabilité

NAV-01 [D] : une mission valide comprend au moins deux points distincts et un ordre explicite. La progression doit visiter tous les points sans s'arrêter définitivement au deuxième. Pour chaque transition, journaliser index courant, distance, critère d'arrivée, durée et décision suivante.

NAV-02 [P] : rayon d'acceptation initial 0,5 m et stabilité pendant 1 s, à adapter à l'incertitude de localisation. Le critère dépend aussi de la vitesse ; traverser rapidement la sphère n'est pas un arrêt réussi. La politique d'arrivée finale et de posé est distincte de l'arrivée à un point intermédiaire.

NAV-03 [D] : les obstacles inconnus sont détectés à partir des mesures disponibles et provoquent une adaptation de trajectoire. La position vérité terrain des obstacles ne doit pas entrer dans le planificateur ou le FC de la recette réaliste.

NAV-04 [D] : YOLO produit des détections d'image ; il ne fournit pas à lui seul une distance métrique ou une localisation. Une caméra RGB et un seul rayon lidar ne couvrent pas tout le volume de vol. Les obstacles sans classe reconnue ne doivent pas être déclarés absents.

NAV-05 [TBD] : sélectionner une source de localisation locale : odométrie visuelle/inertielle, repères visuels ou autre source mesurable. Sans localisation observable, une mission A-vers-B métrique n'est pas recevable. Une source simulée parfaite est autorisée pour diagnostic, avec étiquette « oracle » et exclusion de la recette réaliste.

### Position du lidar : décision ouverte

Vers le bas, le lidar aide l'altitude au-dessus du sol local ; il ne mesure pas la distance des obstacles devant. Vers l'avant, il fournit une distance sur un faisceau étroit ; il ne remplace pas le baromètre pour l'altitude ni une perception de profondeur complète. Une inclinaison exige une projection géométrique et une hypothèse de surface, qui doivent être explicitées. Aucun emplacement n'est déclaré optimal sans essais.

### Enveloppe de démonstration proposée

Jardin illustratif à panneaux statiques, espace dégagé et conditions lumineuses décrites. Une scène « inspirée USTHB » n'est pas une reconstruction géographique ni une autorisation d'essai réel. La vitesse et les distances de sécurité se dimensionnent à partir de la latence totale et du freinage disponible : distance utile supérieure à v·latence + v²/(2·décélération) + marges d'incertitude.

---
## 9. Modèle dynamique et synthèse des gains

MOD-01 [D] : définir un repère monde ENU (z vers le haut) et un repère corps FLU (x avant, y gauche, z haut), ou documenter explicitement toute autre convention. Les transformations ROS et moteur doivent être cohérentes. Les coordonnées fournies dans params.txt ne fixent pas sans ambiguïté le repère ; ne pas déduire le coin moteur du seul numéro.

### Modèle physique de référence

Avec T la somme des quatre poussées en newtons, R la rotation corps vers monde, e3 = [0,0,1]^T et v la vitesse : m·dv/dt = R·e3·T - m·g·e3 - D·v + f_ext. Cette équation est la somme des forces, exprimées dans un même repère. Chaque moteur fournit Ti ; T = T1 + T2 + T3 + T4.

Pour la rotation : J·dω/dt = τ - ω × (J·ω). Le terme croisé décrit le couplage des composantes du moment cinétique dans un repère tournant. Le mélangeur calcule les couples avec les positions mesurées : τ_i = r_i × F_i, plus le couple de réaction de l'hélice. Sa matrice et ses signes doivent être testés avant tout moteur physique.

### Modèle pédagogique LTI autour du vol stationnaire

À petite inclinaison et autour de T0 = mg : Jxx·d²φ/dt² ≈ τx ; Jyy·d²θ/dt² ≈ τy ; Jzz·dr/dt ≈ τz ; m·d²z/dt² ≈ δT. Les fonctions de transfert idéales sont 1/(Jxx·s²), 1/(Jyy·s²), 1/(Jzz·s) pour le taux de lacet et 1/(m·s²) pour z. Ajouter le retard moteur identifié 1/(τm·s+1), délais et filtrage avant synthèse.

MOD-02 [D] : fournir à terme les dérivations progressives, unités, domaine de validité et version discrète. Pour x'=Ax+Bu sous maintien d'ordre zéro : Ad = exp(A·Ts), Bd = intégrale de 0 à Ts de exp(A·t)·B dt. Vérifier la discrétisation sur un cas analytique simple et la stabilité en z, pas seulement en s.

MOD-03 [D] : calculer les gains à partir du modèle identifié ; examiner pôles, amortissement, marges, saturation et incertitudes. Cibles initiales proposées : marge de phase >= 45°, gain >= 6 dB, dépassement angle <= 10 %, établissement à ±5 % <= 1 s pour un petit échelon. Ces critères restent à approuver et ne garantissent pas la stabilité globale.

MOD-04 [D] : séparer erreurs paramétriques fixes, bruit, biais, délais et perturbations. Tester sensibilité et Monte-Carlo avec distributions justifiées ; ne pas inventer de précision en supposant toutes les masses gaussiennes indépendantes.

---
## 10. Énergie, états et risques

ENE-01 [D] : mesurer tension totale par diviseur dimensionné pour la tension maximale 3S (12,6 V) et courant par capteur adapté au courant réel. La broche ADC, le capteur et le conditionnement restent à figer. Calibrer gain/offset contre une référence ; ne pas faire traverser le courant de propulsion par la carte FC.

ENE-02 [D] : estimer la charge restante par intégration du courant, capacité utile et contrôles de cohérence de tension. La relation SOC = SOC0 - intégrale(I dt)/C_utile exige des unités cohérentes ; la tension sous charge seule ne donne pas un pourcentage exact. Le système ne protège pas individuellement les trois cellules.

ENE-03 [D, politique historique à reconfirmer] : avertir à 35 % et demander désarmement à 20 %, sans atterrissage de sécurité automatique. **Une coupure moteur en vol entraîne une chute.** Cette politique est consignée comme demande du porteur, mais n'autorise aucune activation réelle et doit faire l'objet d'une revue explicite avant la phase de vol. Hystérésis, temporisation, incertitude SOC et traitement d'un capteur défaillant sont TBD.

### Machine d'états proposée

BOOT -> SELF_TEST -> BENCH_READY. Les états ARMED_STABLE et ARMED_NAV restent inaccessibles tant que l'interdiction actuelle n'est pas levée explicitement. Toute demande rejetée indique sa cause. Un défaut est mémorisé dans le journal ; un reset ne doit pas provoquer un armement. Le choix final de réaction dépend du défaut et de la phase de vol.

| Risque | Réduction / porte de sortie |
|---|---|
| Signe gyro ou moteur inversé | Rotations connues, mélangeur sur hôte, contrôle des coins ; aucune hélice au banc. |
| BEC en parallèle / surtension | Une seule source positive vérifiée ; masses communes ; mesure avant connexion. |
| Blocage I²C / perte MPU | Timeout borné, invalidation données et restauration bypass vérifiée. |
| Saturation / intégrateur | Limites, anti-windup, journalisation réserve de commande. |
| Perte pilote / compagnon | Politique explicite à approuver ; injection de perte avant vol. |
| Faux sentiment d'évitement | Essais hors classes YOLO et hors faisceau lidar ; refus de mission non observable. |

Les essais motorisés futurs exigent une procédure distincte, un périmètre physique adapté et une autorisation explicite. Ce document n'en tient pas lieu.

---
## 11. Vérification, simulation et preuves

Chaque résultat comporte : identifiant d'essai, commit et SHA source/configuration, matériel ou moteur, paramètres, commande exécutée, données originales, critères, verdict et limites. Les essais échoués restent conservés. Une vidéo seule ne prouve ni le chemin de commande ni le respect du temps réel.

| Test | Exigences principales | Méthode / preuve |
|---|---|---|
| V01 | IF-01, FC-09 | Inspection + mesures alimentation, continuité et broches sans sorties moteur. |
| V02 | IF-02, EST-02 | Boot, coupure complète, reset MPU ; registres et identités avant/après. |
| V03 | EST-01/03 | Calibration gelée, poses indépendantes et rotations signées ; données brutes. |
| V04 | IF-03/04, EST-06 | Lidar I²C à distances connues ; invalidité, retrait, redémarrage. |
| V05 | RT-01/04/06 | Profilage matériel acquisition/estimation/commande sous charge et fautes. |
| V06 | FC-02/03/05/08 | Échelons, relâchement, saturations et signes sur modèle instrumenté. |
| V07 | HMI-01/04, COM-01 | Manette réelle ou replay par le même adaptateur ; durées et perte liaison. |
| V08 | COM-02/03 | CRC, répétition, retard, débit excessif, arrêt du compagnon. |
| V09 | FC-04, EST-05 | Vol vertical simulé avec bruit/retards/capteur perdu ; courbes z/vz. |
| V10 | NAV-01/05 | Missions 2, 3 et n points ; progression complète et localisation documentée. |
| V11 | NAV-03/04 | Obstacles inconnus, limite du faisceau et détection manquée ; aucune vérité terrain cachée. |
| V12 | ENE-01/03 | Mesures de référence, intégration courant, seuils et capteur défaillant sur banc/simulation. |

### Campagne Gazebo proposée

Dix environnements jardin distincts, arrangements d'obstacles et graines enregistrés. Par environnement : cinq missions autonomes avec graines de bruit différentes, soit 50 essais proposés. Ajouter cinq vols manuels scénarisés : décollage, déplacements et posé, uniquement à travers des commandes de manette simulées. Garder les mêmes paramètres pour comparer les révisions ; le porteur construit ses scènes avec une aide ciblée.

Le chemin réaliste est manette -> adaptateur d'entrée -> vrai code FC -> commandes moteurs -> dynamique Gazebo. En navigation : poste sol -> compagnon ROS 2 -> liaison FC -> même code FC -> moteurs. Interdiction d'appliquer une pose, vitesse ou force corrective cachée pour achever une mission. Si le code FC est compilé pour hôte, documenter les shims ; ne pas appeler cela une émulation ESP32-S3 complète.

---
## 12. Recette, résultats et critères proposés

### Mesures à publier pour chaque essai

Consignes et estimations φ/θ/r/z/vz ; erreur RMS et maximum ; sortie PID et intégrateurs ; commandes moteurs et temps de saturation ; âge des capteurs ; période et temps de calcul ; innovations/covariances ; tension/courant/SOC ; index de mission, distance aux points et collisions. Préciser fréquence, unités, repères et synchronisation.

Les vidéos sont enregistrées automatiquement avec les journaux. Une analyse à 10-15 images/s est prévue pour comprendre la trajectoire ; elle ne remplace pas les traces 1 kHz. L'arrêt de session termine proprement l'enregistrement, sauvegarde les fichiers et marque toute fin anormale. La caméra doit provenir du moteur identifié, pas d'une animation reconstruite présentée comme native.

### Seuils de démonstration initiaux, tous proposés

| Domaine | Cible P à revoir |
|---|---|
| Assiette | RMS <= 2° dans la fenêtre stationnaire définie ; pics/transitoires rapportés séparément. |
| Vertical | RMS altitude <= 0,20 m dans l'enveloppe retenue et avec référence disponible. |
| Navigation | Tous les points visités ; erreur terminale <= 0,5 m ; zéro collision sur la campagne définie. |
| Robustesse | Résultats nominaux et perturbés séparés ; aucun essai supprimé des statistiques. |
| Temps réel | Objectifs de la section 6 ; acquisitions perdues, dépassements et latences publiés. |

Une campagne sans collision ne démontre pas un risque nul. Avec zéro échec sur n essais indépendants comparables, la borne supérieure unilatérale à 95 % sur la probabilité d'échec est 1 - 0,05^(1/n), soit environ 5,8 % pour n=50. L'indépendance entre scènes et graines doit être discutée ; ne pas présenter cette borne comme une certification de sûreté.

### Conditions de refus de recette

Absence de source/configuration identifiable ; oracle non déclaré ; caméra ou journaux manquants ; capteur invalide utilisé comme valide ; gain modifié après test sans nouvelle régression ; chemins de commande différents entre démonstration et code livré ; critère ajusté après observation sans nouvelle campagne. Les dérogations sont datées, justifiées et approuvées, jamais masquées.

---
## 13. Développement incrémental et gestion de configuration

| Porte | Livrable minimal | Dépendance d'acceptation |
|---|---|---|
| G0 | Identité des composants, câblage et alimentation | Inspection et mesures sans moteurs. |
| G1 | Capteurs et boot calibration | Intégrité, reset/coupure, montage fixé. |
| G2 | Repères gyro/accéléromètre | Rotations physiques connues ; encore en attente. |
| G3 | Bypass et lidar/BMP I²C | Identités, mesures, fautes et timing. |
| G4 | Estimateur et boucle 1 kHz complète | Rejeu indépendant, charge matérielle et limites. |
| G5 | PID et mélangeur hors actionneurs | Modèle identifié, hôte/Gazebo et saturations. |
| G6 | Manette et stable sans compagnon | Transport compatible et chemin complet de commandes. |
| G7 | Compagnon, missions et perception | Contrat messages, localisation et campagne obstacle. |
| G8 | Préparation à essais motorisés | Revue distincte et levée explicite des restrictions. |

Des étapes ultérieures peuvent être préparées hors ligne sans être acceptées. Si Gk échoue, ses dépendants deviennent invalidés ou bloqués ; corriger Gk puis reconstruire les étapes suivantes sur cette correction. Les anciens essais restent dans l'historique, sans réutiliser une acceptation incompatible.

### Configuration et documentation

Un commit local descriptif et un push GitHub suivent chaque itération modifiant le projet. Aucun historique réécrit ; aucune publication de secrets, environnements locaux ou données personnelles. Les fichiers importants renvoient vers la source autoritative, sans copies divergentes. Les logs originaux sont conservés ; les rapports publics contiennent les résultats relus et leurs limites.

Le cahier des charges évolue par révision. Toute exigence nouvelle doit avoir un identifiant, un statut D/P/TBD, une origine, une méthode de vérification et une dépendance connue. Les branches RP2350/RP2040 restent séparées ; pas de régression involontaire causée par le travail ESP32.

### État de départ au 28 octobre 2026

Calibration et enregistrements inertiels historiques disponibles ; certains contrôles physiques de rotation restent différés. Estimateur préparé sur PC ; PID, mélangeur et vol non acceptés dans la branche Stepwise. Bypass et lidar I²C planifiés, pas encore validés physiquement. Disponibilité actuelle du BMP à reconfirmer. Aucun nouveau test matériel exécuté pour produire ce document.

---
## 14. Références, traçabilité et revue

### Sources internes autoritatives

S01 : instructions du porteur et docs/PLAN.md de la branche codex/stepwise-esp32, incluant les choix bypass et lidar I²C du 27 octobre. S02 : knowledge/state.json et decisions/0001-current-contract.md pour les restrictions et l'état des preuves. S03 : guides et rapports FIFO1000, calibration multi-pose et gyro-frame cités dans le plan. S04 : fichier utilisateur params.txt, entrées de simulation, non qualifiées comme métrologie réelle. S05 : demandes historiques sur Xbox, ROS 2, batterie et contrôle, avec priorité aux décisions les plus récentes.

### Documents fabricants à utiliser lors de l'implémentation

Espressif ESP32-S3 : https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf

TDK MPU6050 : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

Bosch BMP280 : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

Nooploop TOFSense-F V2.0 : https://ftp.nooploop.com/downloads/tofsense/TOFSense-F_Datasheet_V2.0_en.pdf ; connectique et protocole : https://support.nooploop.com/tofsense/hardware/ et https://support.nooploop.com/tofsense/protocol/

### Origine des familles d'exigences

FC/HMI : S01 et S05 ; EST/RT : S01-S03 ; IF : S01 et documents fabricants ; MOD : S04 et besoin pédagogique ; NAV/COM : S05 avec critères P d'ingénierie ; ENE : S05 avec politique historique à reconfirmer. Les valeurs de recette P de cette révision sont proposées par le rédacteur, et ne proviennent ni d'une certification ni d'un essai de vol.

### Revue de gel avant implémentation

Confirmer les axes et coins moteurs ; la compatibilité Xbox ; le choix du lidar bas/avant ; les budgets et critères P ; la politique perte de liaison et batterie ; les mesures de propulsion ; le transport compagnon. L'absence de réponse garde les points TBD ouverts et n'autorise aucune action physique.

**Approbation proposée :** porteur du projet, date, révision et liste des réserves. Ce champ reste non signé. La présente révision constitue une base de discussion et de développement vérifiable, pas une déclaration d'aptitude au vol.

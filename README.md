# Mobil gépek mechatronikája projektfeladat

---

## 1. GitHub használata

### 1.1. Lokális GitHub repo létrehozása - ezt egyszer kell megcsinálni setupoláskor

Amikor először le akarjátok tölteni erről a GitHub repo-ról a file-okat lokálisan a gépetekre, mindenekelőtt be kell állítani néhány dolgot.

**Felhasználónév és email létrehozása**

A terminálba a következő parancsot kell lefuttatni:

```
$ git config --global user.name "Keresztnev Vezeteknev"
$ git config --global user.email emailcim@domain.hu
```

Ezek azért kellenek, hogy amikor egyes változtatásokat hozunk létre, azt lássuk, hogy ki csinálta.

**A GitHub repo klónozása lokális repoba**

Ehhez mindenekelőtt szükség van egy ún. _Personal Access Token_-re. Ezt a GitHub-ban lehet beállítani a következő módon:

- Be kell lépni a: **Settings --> Developer Settings --> Personal access token --> Tokens(classic) --> Generate new token (classic)**
- Itt meg lehet adni egy **Note**-ot magadnak, de ami fontos, hogy az **Expiration** legyen legalább 90 nap, illetve a **Select scopes**-nál a **repo** legyen kipipálva
- Majd **Generate token** és az így létrejött token kódot ki kell másolni! FONTOS, mert el fog tűnni

A token használatával le lehet klónozni a GitHub repo-t:

```
$ git clone https://<USERNAME>:<TOKEN>@github.com/BalassaHodi/mgm-project.git
```

Itt a `<USERNAME>` a GitHub felhasználónevetek, a `<TOKEN>` pedig az előbb kimásolt token.

Majd ezt a tokent el kell menteni, hogy később ne kelljen megadni:

```
$ git config --global credential.helper store
```

### 1.2. GitHub repo-ban levő módosítások átmásolása lokális repoba

Mielőtt nekikezdenétek dolgozni a feladaton, először is mindig le kell "húzni" a legkorábbi módosításokat ami a GitHub repo `main` branch-ben vannak. Ezt az alábbi módon lehet megtenni:

```
$ git checkout main
$ git pull origin main
```

Ezzel a lokális repo `main` branch-je a legfrisebb állapotában van

### 1.3. Lokális munkavégzés egy saját branch-ben

Mindig ha valamit csinálni szeretnétek a projektfeladatban, azt sose a `main`-en belül tegyétek. Mindig érdemes egy branch-et létrehozni rá, amiben elvégzitek a módosításokat, majd azt feltöltitek a GitHub repo-ba, ahonnan majd átrakjuk a `main`-be.

Egy lokális branch létrehozása:

```
$ git checkout -b branch-name
```

A branch nevét érdemes úgy választani meg, hogy az rámutasson arra egy szóban, hogy mit csináltatok éppen.

Miután befejeztétek az adott feladatotokat, a módosított file-okat először fel kell küldeni a "staged" file-ok közé:

```
$ git add file-name
```

Vagy hasonló módon, ha minden módosított file-t fel akarsz rakni:

```
$ git add .
```

Ezután ha már végeztünk egy adott feladaton belül egy módosítással, azt "commit"-olni lehet. A commitokat érdemes úgy létrehozni, hogy mindig egy adott prokekthez/feladathoz/munkához tartozó fileok és módosítások legyenek benne. Ne legyen benne túl sok különböző tematkájú munka, azokhoz érdemes külön commit-ok. A commmit menete:

```
$ git commit -m "Ide jon egy rovid uzenet ami leirja az adott commit-ban mit csinaltatok"
```

### 1.4. Lokális branch felküldése GitHub repo-ba és Pull Request létrehozása

Miután végeztetek egy adott branch-el, egy adott feladattal, amihez külön létrehoztatok egy branch-et, és úgy gondoljátok ezt a többieknek is érdemes látniuk, akkor kell felküldeni ezt a branch-et a GitHub repoba az alábbi módon:

```
$ git push origin branch-name
```

Miután ez megvolt, hogy a többiek lássák a változtatásokat, és közösen elfogadjuk azt, létre kell hozni egy _Pull Request_-et GitHub-on. Ez az alábbi módon történik:

- Valószínűleg megjelenik egy **Compare & pull request** gomb, ha nem: **Pull Request --> New pull request**
- Meg kell adni:
  - **Title:** Címet adni
  - **Description:** Rövid magyarázat a változtatásokról
  - **Reviewers:** Ide meg lehet adni azt akinek ezt érdemes megnéznie (mindenkit akár)
- majd **Create Pull Request**

Ezt a _Pull request_-et a többiek megnézhetik, kommentelhetnek alá, kérhetnek változásokat, de majd ha el lett fogadva jónak, akkor össze lehet fűzni a `main` branch-el a **Merge pull request** gombra kattintva.

Miután ez megvolt le lehet törölni a githubban a branchet, illetve lokálisan is le lehet törölni az adott branchet:

```
$ git checkout main
$ git branch -d branch_name_that_you_want_to_delete
```

### 1.5. Lokális repo frissítése, munkafolyamat ismétlése

Miután GitHub-ban merge-elve lett a branch és a main, azután hogy a legfrissebb legyen a lokális `main`, így megismételjük elölről a folyamatot az **1.2.** ponttól.

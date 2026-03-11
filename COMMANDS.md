# esp_at — Comandos úteis

Substitua `<github-user>` pelo seu usuário do GitHub.

---

## Primeiro push (repo recém-criado no GitHub com LICENSE automática)

```bash
git remote add origin https://github.com/<github-user>/esp-at.git
git pull origin main --allow-unrelated-histories
git checkout --ours LICENSE
git add LICENSE
git commit -m "merge: keep local LICENSE"
git push -u origin main
git push origin v1.0.0
```

---

## Fluxo de atualização (nova versão)

```bash
# 1. Faça as alterações nos arquivos

# 2. Commit
git add .
git commit -m "feat: descrição da mudança"

# 3. Nova tag de versão
git tag v1.1.0

# 4. Push
git push origin main
git push origin v1.1.0
```

---

## Usar como submodule em outro projeto

```bash
git submodule add https://github.com/<github-user>/esp-at.git components/esp_at
git submodule update --init
```

---

## Clonar projeto que usa este submodule

```bash
git clone --recurse-submodules <url-do-projeto>
# ou, após clonar sem submodules:
git submodule update --init
```

---

## Atualizar submodule para versão mais nova

```bash
cd components/esp_at
git pull origin main
cd ../..
git add components/esp_at
git commit -m "chore: update esp_at submodule"
```

---

## Instalar via IDF Component Manager

```bash
idf.py add-dependency "<github-user>/esp_at>=1.0.0"
```

FROM node:14.20.0 as builder

WORKDIR /app
COPY package.json ./
RUN npm install
COPY . ./
RUN npm run build
RUN npx pkg ./node_modules/@import-meta-env/cli/bin/import-meta-env.js -t node14-alpine -o import-meta-env

FROM nginx:stable-alpine as production-stage

WORKDIR /app

COPY --from=builder /app/build ./build
COPY --from=builder /app/import-meta-env ./import-meta-env

COPY .env.example ./
COPY nginx.conf /etc/nginx/nginx.conf
COPY entrypoint.sh ./entrypoint.sh

RUN chmod +x entrypoint.sh

ENTRYPOINT ["sh", "entrypoint.sh"]

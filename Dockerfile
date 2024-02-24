FROM rust:1.70 AS builder
WORKDIR /usr/client_mqtt/
COPY  Cargo.toml Cargo.lock src/ ./
RUN cargo build --release

FROM alpine
WORKDIR /app
COPY --from=builder /usr/client_mqtt/target/release/mqtt-example .
CMD ["./mqtt-example"]
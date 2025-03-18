use crate::data_types::{DataResponse, SharedDataBuffer};
use actix_web::{get, web, App, HttpResponse, HttpServer, Responder};
use actix_files::Files;
use std::error::Error;

pub struct WebServer {
    addr: String,
    buffer: SharedDataBuffer,
}

#[get("/api/data")]
async fn get_data(buffer: web::Data<SharedDataBuffer>) -> impl Responder {
    let buffer = buffer.read().await;
    let response = DataResponse::from_buffer(&buffer);
    HttpResponse::Ok().json(response)
}

impl WebServer {
    pub fn new(addr: &str, buffer: SharedDataBuffer) -> Self {
        Self {
            addr: addr.to_string(),
            buffer,
        }
    }

    pub async fn start(&self) -> Result<(), Box<dyn Error>> {
        let buffer = self.buffer.clone();
        
        println!("Starting web server on {}", self.addr);
        
        HttpServer::new(move || {
            App::new()
                .app_data(web::Data::new(buffer.clone()))
                .service(get_data)
                .service(Files::new("/", "./static").index_file("index.html"))
        })
        .bind(&self.addr)?
        .run()
        .await?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_types::{DataBuffer, SensorData};
    use std::sync::Arc;
    use tokio::sync::RwLock;

    #[tokio::test]
    async fn test_web_server_creation() {
        let buffer = Arc::new(RwLock::new(DataBuffer::new(1000)));
        let server = WebServer::new("127.0.0.1:8080", buffer);
        assert_eq!(server.addr, "127.0.0.1:8080");
    }

    #[actix_web::test]
    async fn test_get_data_endpoint() {
        let buffer = Arc::new(RwLock::new(DataBuffer::new(1000)));
        
        // Add some test data
        {
            let mut buffer_write = buffer.write().await;
            buffer_write.add_sample(SensorData {
                timestamp: 1.0,
                adc_values: vec![100, 200, 300],
            });
        }

        let app = actix_web::test::init_service(
            App::new()
                .app_data(web::Data::new(buffer))
                .service(get_data)
        ).await;

        let req = actix_web::test::TestRequest::get()
            .uri("/api/data")
            .to_request();
        let resp = actix_web::test::call_service(&app, req).await;
        assert!(resp.status().is_success());
    }
}


fn main() {
    #[cfg(any(target_os = "linux"))]
    {
        println!("cargo:rustc-link-arg=-z");
        println!("cargo:rustc-link-arg=nostart-stop-gc");
    }
}

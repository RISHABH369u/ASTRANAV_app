plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.compose)
}

android {
    namespace = "com.rishabh.astranav"

    compileSdk = 37

    defaultConfig {
        applicationId = "com.rishabh.astranav"

        minSdk = 26
        targetSdk = 37

        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner =
            "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        debug {
            isMinifyEnabled = false
        }

        release {
            isMinifyEnabled = false

            proguardFiles(
                getDefaultProguardFile(
                    "proguard-android-optimize.txt"
                ),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    buildFeatures {
        buildConfig = true
        compose = true
    }

    packaging {
        resources {
            excludes += setOf(
                "META-INF/AL2.0",
                "META-INF/LGPL2.1",
                "META-INF/DEPENDENCIES",
                "META-INF/LICENSE",
                "META-INF/LICENSE.txt",
                "META-INF/NOTICE",
                "META-INF/NOTICE.txt",
                "osmosis-plugins.conf"
            )
        }
    }
}

dependencies {

    // ============================================================
    // ANDROID CORE
    // ============================================================

    implementation("androidx.core:core-ktx:1.18.0")
    implementation("androidx.appcompat:appcompat:1.7.1")

    implementation("com.google.guava:listenablefuture:9999.0-empty-to-avoid-conflict-with-guava")

    implementation("com.google.android.material:material:1.13.0")

    implementation("androidx.activity:activity-ktx:1.11.0")
    implementation("androidx.fragment:fragment-ktx:1.8.9")

    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.10.0")
    implementation("androidx.lifecycle:lifecycle-viewmodel-ktx:2.10.0")



    // ============================================================
    // COMPOSE UI
    // ============================================================

    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.compose.ui)
    implementation(libs.androidx.compose.ui.graphics)
    implementation(libs.androidx.compose.ui.tooling.preview)
    implementation(libs.androidx.compose.material3)
    debugImplementation(libs.androidx.compose.ui.tooling)

    // ============================================================
    // XML UI
    // ============================================================

    implementation(
        "androidx.constraintlayout:constraintlayout:2.2.1"
    )

    implementation(
        "androidx.recyclerview:recyclerview:1.4.0"
    )

    implementation(
        "androidx.cardview:cardview:1.0.0"
    )


    // ============================================================
    // NAVIGATION COMPONENT
    // XML + FRAGMENT BASED
    // ============================================================

    implementation(
        "androidx.navigation:navigation-fragment-ktx:2.9.3"
    )

    implementation(
        "androidx.navigation:navigation-ui-ktx:2.9.3"
    )


    // ============================================================
    // COROUTINES
    // ============================================================

    implementation(
        "org.jetbrains.kotlinx:kotlinx-coroutines-android:1.10.2"
    )


    // ============================================================
    // GOOGLE LOCATION / GNSS
    // ============================================================

    implementation(
        "com.google.android.gms:play-services-location:21.3.0"
    )


    // ============================================================
    // OPENSTREETMAP
    // ============================================================

    implementation(
        "org.osmdroid:osmdroid-android:6.1.20"
    )


    // ============================================================
    // OFFLINE OSM PBF
    // ============================================================

    implementation(
        "org.openstreetmap.osmosis:osmosis-pbf2:0.48.3"
    ) {
        exclude(
            group = "org.springframework",
            module = "spring-jdbc"
        )
    }


    // ============================================================
    // JSON
    // ============================================================

    implementation(
        "com.google.code.gson:gson:2.13.1"
    )


    // ============================================================
    // MATRIX / NUMERICAL COMPUTATION
    // ESKF / KALMAN / TRANSFORMATIONS
    // ============================================================

    implementation(
        "org.ejml:ejml-all:0.44.0"
    )


    // ============================================================
    // ON-DEVICE ML
    // TCN SPEED ESTIMATOR
    // ============================================================

    implementation(
        "com.microsoft.onnxruntime:onnxruntime-android:1.22.0"
    )


    // ============================================================
    // TESTING
    // ============================================================

    testImplementation(
        "junit:junit:4.13.2"
    )

    testImplementation(
        "org.jetbrains.kotlinx:kotlinx-coroutines-test:1.10.2"
    )

    testImplementation(
        "com.microsoft.onnxruntime:onnxruntime:1.22.0"
    )

    androidTestImplementation(
        "androidx.test.ext:junit:1.3.0"
    )

    androidTestImplementation(
        "androidx.test.espresso:espresso-core:3.7.0"
    )


    implementation("io.github.sceneview:sceneview:2.2.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.4")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.8.1")
}